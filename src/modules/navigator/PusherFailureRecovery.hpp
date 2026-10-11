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
#pragma once

#include <cmath>
#include <uORB/topics/pusher_failure_status.h>

// Sequencing only: Navigator owns admission checks, navigation setpoints and commands.
class PusherFailureRecovery
{
public:
	using Status = pusher_failure_status_s;

	struct Input {
		bool armed;
		bool landed;
		bool hold;
		bool land;
		bool multicopter;
		float height;
	};

	bool start(float recovery_height)
	{
		if (_state != Status::STATE_IDLE || !std::isfinite(recovery_height) || recovery_height <= 0.f) {
			return false;
		}

		_recovery_height = recovery_height;
		_state = Status::STATE_GLIDING;
		return true;
	}

	void update(const Input &input)
	{
		if (!input.armed) {
			_state = Status::STATE_IDLE;
			_recovery_height = NAN;
			_land_mode_seen = false;
			return;
		}

		if (_state == Status::STATE_IDLE || _state == Status::STATE_ABORTED || _state == Status::STATE_LANDED) {
			return;
		}

		if (input.landed) {
			_state = Status::STATE_LANDED;
			return;
		}

		// Respect pilot takeover and other failsafe modes. Never re-enable the pusher.
		if ((!input.hold && !input.land) || (_land_mode_seen && !input.land)) {
			_state = Status::STATE_ABORTED;
			return;
		}

		if (input.multicopter) {
			_state = Status::STATE_LANDING;
			_land_mode_seen |= input.land;
			return;
		}

		if (_state == Status::STATE_GLIDING
		    && (!input.hold || !std::isfinite(input.height) || input.height <= _recovery_height)) {
			// Loss of the height reference must not allow an indefinite gliding descent.
			_state = Status::STATE_QUADCHUTE;
		}
	}

	uint8_t state() const { return _state; }

	bool pusherFailed() const { return _state != Status::STATE_IDLE; }

	float recoveryHeight() const { return _recovery_height; }

private:
	uint8_t _state{Status::STATE_IDLE};
	bool _land_mode_seen{false};
	float _recovery_height{NAN};
};
