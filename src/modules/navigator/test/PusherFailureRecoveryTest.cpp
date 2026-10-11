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
#include "PusherFailureRecovery.hpp"

using Status = pusher_failure_status_s;

class PusherFailureRecoveryTest : public ::testing::Test
{
protected:
	PusherFailureRecovery recovery;
	PusherFailureRecovery::Input input{true, false, true, false, false, 100.f};
};

TEST_F(PusherFailureRecoveryTest, GlideQuadchuteLandAndReset)
{
	ASSERT_TRUE(recovery.start(50.f));
	recovery.update(input);
	EXPECT_EQ(recovery.state(), Status::STATE_GLIDING);
	input.height = 50.f;
	recovery.update(input);
	EXPECT_EQ(recovery.state(), Status::STATE_QUADCHUTE);
	input.multicopter = true;
	recovery.update(input);
	EXPECT_EQ(recovery.state(), Status::STATE_LANDING);
	input.hold = false;
	input.land = true;
	recovery.update(input);
	EXPECT_EQ(recovery.state(), Status::STATE_LANDING);
	input.landed = true;
	recovery.update(input);
	EXPECT_EQ(recovery.state(), Status::STATE_LANDED);
	EXPECT_TRUE(recovery.pusherFailed());
	input.armed = false;
	recovery.update(input);
	EXPECT_EQ(recovery.state(), Status::STATE_IDLE);
	EXPECT_FALSE(recovery.pusherFailed());
	EXPECT_TRUE(recovery.start(50.f));
}

TEST_F(PusherFailureRecoveryTest, InvalidHeightImmediatelyRequestsQuadchute)
{
	ASSERT_TRUE(recovery.start(50.f));
	input.height = NAN;
	recovery.update(input);
	EXPECT_EQ(recovery.state(), Status::STATE_QUADCHUTE);
}

TEST_F(PusherFailureRecoveryTest, BelowThresholdDoesNotWaitForOrbit)
{
	ASSERT_TRUE(recovery.start(50.f));
	input.height = 30.f;
	recovery.update(input);
	EXPECT_EQ(recovery.state(), Status::STATE_QUADCHUTE);
}

TEST_F(PusherFailureRecoveryTest, QuadchuteDoesNotRevertWhenHeightIncreases)
{
	ASSERT_TRUE(recovery.start(50.f));
	input.height = 49.f;
	recovery.update(input);
	input.height = 80.f;
	recovery.update(input);
	EXPECT_EQ(recovery.state(), Status::STATE_QUADCHUTE);
}

TEST_F(PusherFailureRecoveryTest, PilotTakeoverKeepsPusherFailedUntilDisarm)
{
	ASSERT_TRUE(recovery.start(50.f));
	input.hold = false;
	recovery.update(input);
	EXPECT_EQ(recovery.state(), Status::STATE_ABORTED);
	EXPECT_TRUE(recovery.pusherFailed());
	input.hold = true;
	recovery.update(input);
	EXPECT_EQ(recovery.state(), Status::STATE_ABORTED);
	EXPECT_FALSE(recovery.start(60.f));
	input.armed = false;
	recovery.update(input);
	EXPECT_FALSE(recovery.pusherFailed());
}

TEST_F(PusherFailureRecoveryTest, OtherQuadchuteCanLandBeforeHeightThreshold)
{
	ASSERT_TRUE(recovery.start(50.f));
	input.multicopter = true;
	recovery.update(input);
	EXPECT_EQ(recovery.state(), Status::STATE_LANDING);
}

TEST_F(PusherFailureRecoveryTest, PilotCanLeaveLandForHold)
{
	ASSERT_TRUE(recovery.start(50.f));
	input.multicopter = true;
	input.hold = false;
	input.land = true;
	recovery.update(input);
	input.hold = true;
	input.land = false;
	recovery.update(input);
	EXPECT_EQ(recovery.state(), Status::STATE_ABORTED);
	EXPECT_TRUE(recovery.pusherFailed());
}

TEST_F(PusherFailureRecoveryTest, InvalidAndDuplicateTriggersDoNotChangeRecoveryHeight)
{
	EXPECT_FALSE(recovery.start(NAN));
	EXPECT_FALSE(recovery.start(0.f));
	EXPECT_FALSE(recovery.start(-1.f));
	ASSERT_TRUE(recovery.start(50.f));
	EXPECT_FALSE(recovery.start(100.f));
	EXPECT_FLOAT_EQ(recovery.recoveryHeight(), 50.f);
}
