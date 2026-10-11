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
#include <gtest/gtest.h>
#include "navigator.h"
#include "support/navigator_dataman_test.h"
#include <lib/parameters/param.h>
#include <uORB/uORBManager.hpp>
#include <memory>

class PusherFailureTestPeer
{
public:
	static void request(Navigator &navigator) { navigator._pusher_failure_requested.store(true); }

	static void update(Navigator &navigator) { navigator.update_pusher_failure(); }

	static uint8_t state(const Navigator &navigator) { return navigator._pusher_failure.state(); }
};

class PusherFailureIntegrationTest : public ::testing::Test
{
protected:
	using Status = pusher_failure_status_s;

	static void SetUpTestSuite()
	{
		uORB::Manager::initialize();
		(void)navigatorDatamanRuntime();
	}

	void SetUp() override
	{
		param_control_autosave(false);
		const char *names[] = {"SYS_FAILURE_EN", "VT_TYPE", "COM_QC_ACT", "VT_FW_QC_HMAX"};
		const int32_t values[] = {1, 2, 1, 50};

		for (unsigned i = 0; i < 4; ++i) {
			params[i] = param_find(names[i]);
			ASSERT_NE(params[i], PARAM_INVALID);
			ASSERT_EQ(param_get(params[i], &backups[i]), 0);
			ASSERT_EQ(param_set_no_notification(params[i], &values[i]), 0);
		}

		nav = std::make_unique<Navigator>();
		auto &vehicle = *nav->get_vstatus();
		vehicle.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
		vehicle.is_vtol = true;
		vehicle.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
		vehicle.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
		auto &local = *nav->get_local_position();
		local.xy_valid = true;
		local.xy_global = true;
		local.z_valid = true;
		local.z = -100.f;
		local.dist_bottom_valid = true;
		local.dist_bottom = 100.f;
		auto &global = *nav->get_global_position();
		global.lat = 47.397742;
		global.lon = 8.545594;
		global.alt = 600.f;
		auto &home = *nav->get_home_position();
		home.valid_alt = true;
		home.valid_lpos = true;
		home.z = 0.f;
		auto &sp = nav->get_position_setpoint_triplet()->current;
		sp.valid = true;
		sp.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
		sp.lat = global.lat;
		sp.lon = global.lon;
		sp.alt = global.alt;
		sp.loiter_radius = 80.f;
		refresh();
		vehicle_command_s command{};

		while (command_sub.update(&command)) {}
	}

	void TearDown() override
	{
		nav.reset();

		for (unsigned i = 0; i < 4; ++i) {
			if (params[i] != PARAM_INVALID) {
				param_set_no_notification(params[i], &backups[i]);
			}
		}
	}

	void refresh()
	{
		nav->get_local_position()->timestamp = hrt_absolute_time();
		nav->get_global_position()->timestamp = hrt_absolute_time();
	}

	void trigger()
	{
		PusherFailureTestPeer::request(*nav);
		PusherFailureTestPeer::update(*nav);
	}

	std::unique_ptr<Navigator> nav;
	param_t params[4] {PARAM_INVALID, PARAM_INVALID, PARAM_INVALID, PARAM_INVALID};
	int32_t backups[4] {};
	uORB::Subscription command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription status_sub{ORB_ID(pusher_failure_status)};
};

TEST_F(PusherFailureIntegrationTest, PublishesGlideThenImmediateTransitionThenLand)
{
	trigger();
	EXPECT_EQ(PusherFailureTestPeer::state(*nav), Status::STATE_GLIDING);
	EXPECT_TRUE(nav->get_position_setpoint_triplet()->current.gliding_enabled);
	EXPECT_FLOAT_EQ(nav->get_position_setpoint_triplet()->current.loiter_radius, 80.f);
	Status status{};
	ASSERT_TRUE(status_sub.update(&status));
	EXPECT_TRUE(status.pusher_failed);
	EXPECT_TRUE(status.terrain_height);
	EXPECT_FLOAT_EQ(status.recovery_height, 50.f);
	vehicle_command_s command{};
	EXPECT_FALSE(command_sub.update(&command));

	nav->get_local_position()->dist_bottom = 50.f;
	PusherFailureTestPeer::update(*nav);
	ASSERT_TRUE(command_sub.update(&command));
	EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
	EXPECT_FLOAT_EQ(command.param1, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
	EXPECT_FLOAT_EQ(command.param2, 1.f);
	PusherFailureTestPeer::update(*nav);
	EXPECT_FALSE(command_sub.update(&command)); // no command flooding

	nav->get_vstatus()->vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	PusherFailureTestPeer::update(*nav);
	ASSERT_TRUE(command_sub.update(&command));
	EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_NAV_LAND);
	EXPECT_TRUE(std::isnan(command.param5));
	EXPECT_TRUE(std::isnan(command.param6));
	EXPECT_FALSE(nav->get_position_setpoint_triplet()->current.gliding_enabled);

	nav->get_vstatus()->arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	PusherFailureTestPeer::update(*nav);
	ASSERT_TRUE(status_sub.update(&status));
	EXPECT_FALSE(status.pusher_failed);
}

TEST_F(PusherFailureIntegrationTest, RejectsGroundAndWrongModeWithoutInhibitingPusher)
{
	nav->get_land_detected()->landed = true;
	trigger();
	EXPECT_EQ(PusherFailureTestPeer::state(*nav), Status::STATE_IDLE);
	nav->get_land_detected()->landed = false;
	nav->get_vstatus()->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	trigger();
	EXPECT_EQ(PusherFailureTestPeer::state(*nav), Status::STATE_IDLE);
	EXPECT_FALSE(nav->get_position_setpoint_triplet()->current.gliding_enabled);
}

TEST_F(PusherFailureIntegrationTest, RejectsDisabledInjectionAndWrongQuadchuteAction)
{
	int32_t disabled = 0;
	param_set_no_notification(params[0], &disabled);
	trigger();
	EXPECT_EQ(PusherFailureTestPeer::state(*nav), Status::STATE_IDLE);
	int32_t enabled = 1;
	param_set_no_notification(params[0], &enabled);
	param_set_no_notification(params[2], &disabled);
	trigger();
	EXPECT_EQ(PusherFailureTestPeer::state(*nav), Status::STATE_IDLE);
}

TEST_F(PusherFailureIntegrationTest, UsesHomeHeightAndRecoversOnLossOfReference)
{
	nav->get_local_position()->dist_bottom_valid = false;
	trigger();
	Status status{};
	ASSERT_TRUE(status_sub.update(&status));
	EXPECT_FALSE(status.terrain_height);
	EXPECT_FLOAT_EQ(status.height, 100.f);
	nav->get_home_position()->valid_lpos = false;
	PusherFailureTestPeer::update(*nav);
	vehicle_command_s command{};
	ASSERT_TRUE(command_sub.update(&command));
	EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
}

TEST_F(PusherFailureIntegrationTest, RejectsStalePosition)
{
	nav->get_global_position()->timestamp = 0;
	trigger();
	EXPECT_EQ(PusherFailureTestPeer::state(*nav), Status::STATE_IDLE);
}

TEST_F(PusherFailureIntegrationTest, NavigationLossRequestsRecoveryWithoutPublishingNanAltitude)
{
	trigger();
	nav->get_global_position()->alt = NAN;
	PusherFailureTestPeer::update(*nav);
	vehicle_command_s command{};
	ASSERT_TRUE(command_sub.update(&command));
	EXPECT_EQ(command.command, vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION);
	EXPECT_TRUE(std::isfinite(nav->get_position_setpoint_triplet()->current.alt));
}
