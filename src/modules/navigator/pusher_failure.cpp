/***************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include "navigator.h"

#include <lib/parameters/param.h>
#include <uORB/topics/vtol_vehicle_status.h>

namespace
{
bool parameter_equals(const char *name, int32_t expected)
{
	int32_t value = -1;
	return param_get(param_find(name), &value) == PX4_OK && value == expected;
}
}

void Navigator::update_pusher_failure()
{
	using Status = pusher_failure_status_s;
	const hrt_abstime now = hrt_absolute_time();
	const bool armed = _vstatus.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	const bool hold = _vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
	const bool land = _vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
	const bool multicopter = _vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
				 && !_vstatus.in_transition_mode;
	const bool local_valid = _local_pos.timestamp != 0 && now - _local_pos.timestamp < 1_s
				 && _local_pos.z_valid && PX4_ISFINITE(_local_pos.z);
	const bool terrain_height = local_valid && _local_pos.dist_bottom_valid
				    && PX4_ISFINITE(_local_pos.dist_bottom) && _local_pos.dist_bottom >= 0.f;
	float height = NAN;
	const bool position_valid = local_valid && _local_pos.xy_valid && _local_pos.xy_global
				    && _global_pos.timestamp != 0 && now - _global_pos.timestamp < 1_s
				    && PX4_ISFINITE(_global_pos.lat) && PX4_ISFINITE(_global_pos.lon)
				    && PX4_ISFINITE(_global_pos.alt);

	if (terrain_height) {
		height = _local_pos.dist_bottom;

	} else if (local_valid && _home_pos.valid_alt && _home_pos.valid_lpos && PX4_ISFINITE(_home_pos.z)) {
		height = _home_pos.z - _local_pos.z;
	}

	bool requested = true;

	if (_pusher_failure_requested.compare_exchange(&requested, false)) {
		int32_t recovery_height = 0;
		param_get(param_find("VT_FW_QC_HMAX"), &recovery_height);
		// Parameter values: SYS_FAILURE_EN enabled, VT_TYPE standard VTOL, COM_QC_ACT Land.
		const bool configured = parameter_equals("SYS_FAILURE_EN", 1) && parameter_equals("VT_TYPE", 2)
					&& parameter_equals("COM_QC_ACT", 1) && recovery_height > 0;
		const bool flying = armed && !_land_detected.landed && _vstatus.is_vtol && !_vstatus.in_transition_mode
				    && _vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

		if (configured && flying && position_valid && PX4_ISFINITE(height) && hold && !_vstatus.failsafe
		    && _pos_sp_triplet.current.valid && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER
		    && _pusher_failure.start(static_cast<float>(recovery_height))) {
			_pusher_failure_loiter = _pos_sp_triplet.current;
			_pusher_failure_loiter.loiter_pattern = position_setpoint_s::LOITER_TYPE_ORBIT;
			_pusher_failure_loiter.gliding_enabled = true;
			_pusher_failure_last_command = 0;
			mavlink_log_critical(&_mavlink_log_pub, "Pusher failure injected: gliding to %ld m (%s)\t",
					     (long)recovery_height, terrain_height ? "ground" : "home");

		} else {
			PX4_WARN("Pusher failure rejected: need SYS_FAILURE_EN=1, VT_TYPE=2, COM_QC_ACT=1, VT_FW_QC_HMAX>0,");
			PX4_WARN("armed FW Hold, valid position/height, no failsafe and no prior pusher failure");
		}
	}

	const uint8_t previous_state = _pusher_failure.state();
	_pusher_failure.update({armed, _land_detected.landed, hold, land, multicopter, position_valid ? height : NAN});
	const uint8_t state = _pusher_failure.state();

	if (state != previous_state) {
		_pusher_failure_last_command = 0;
		PX4_INFO("Pusher recovery state %u", state);
	}

	if ((state == Status::STATE_GLIDING || state == Status::STATE_QUADCHUTE) && hold && !multicopter) {
		// Keep the captured circle even if a reposition command arrives during recovery.
		_pos_sp_triplet.current = _pusher_failure_loiter;
		_pos_sp_triplet.current.timestamp = now;

		if (PX4_ISFINITE(_global_pos.alt)) {
			_pos_sp_triplet.current.alt = _global_pos.alt;
		}

		_pos_sp_triplet.previous.valid = false;
		_pos_sp_triplet.next.valid = false;
		set_position_setpoint_triplet_updated();
	}

	// A new mode or pilot takeover must not inherit the temporary gliding setpoint.
	if (state != Status::STATE_GLIDING && state != Status::STATE_QUADCHUTE
	    && (previous_state == Status::STATE_GLIDING || previous_state == Status::STATE_QUADCHUTE) && hold) {
		_pos_sp_triplet.current.gliding_enabled = false;
		set_position_setpoint_triplet_updated();
	}

	if (state == Status::STATE_QUADCHUTE || (state == Status::STATE_LANDING && !land)) {
		if (_pusher_failure_last_command == 0 || now - _pusher_failure_last_command >= 1_s) {
			vehicle_command_s command{};

			if (state == Status::STATE_QUADCHUTE) {
				command.command = vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION;
				command.param1 = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
				command.param2 = 1.f; // Immediate transition (quadchute), not a normal back transition.

			} else {
				command.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
				command.param4 = NAN;
				command.param5 = static_cast<double>(NAN);
				command.param6 = static_cast<double>(NAN);
				command.param7 = NAN;
			}

			publish_vehicle_command(command);
			_pusher_failure_last_command = now;
		}
	}

	if (_pusher_failure.pusherFailed() || previous_state != Status::STATE_IDLE) {
		Status status{};
		status.timestamp = now;
		status.state = state;
		status.pusher_failed = _pusher_failure.pusherFailed();
		status.recovery_height = _pusher_failure.recoveryHeight();
		status.height = height;
		status.terrain_height = terrain_height;
		_pusher_failure_pub.publish(status);
	}
}
