// Copyright 2026 scramble-robot
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
#include <gtest/gtest.h>

#include <lifecycle_msgs/msg/state.hpp>

#include "motor_control_app/shot_auto_start.hpp"

namespace {

using lifecycle_msgs::msg::State;
using motor_control_app::shot_auto_start::AutoStartAction;
using motor_control_app::shot_auto_start::decideAutoStartAction;
using motor_control_app::shot_auto_start::shouldFailSafeControllableTimeout;

// /gpio/controllable 未受信（enable_gpio_ref=false 等）は周期リトライにフォールバック
TEST(ShotAutoStart, UnconfiguredWithoutControllableFallsBackToConfigure) {
  EXPECT_EQ(decideAutoStartAction(State::PRIMARY_STATE_UNCONFIGURED, false, false),
            AutoStartAction::kConfigure);
}

TEST(ShotAutoStart, UnconfiguredWhileEstopPressedWaits) {
  EXPECT_EQ(decideAutoStartAction(State::PRIMARY_STATE_UNCONFIGURED, true, false),
            AutoStartAction::kWaitEstopRelease);
}

TEST(ShotAutoStart, UnconfiguredAfterEstopReleaseConfigures) {
  EXPECT_EQ(decideAutoStartAction(State::PRIMARY_STATE_UNCONFIGURED, true, true),
            AutoStartAction::kConfigure);
}

TEST(ShotAutoStart, InactiveWithoutControllableActivates) {
  EXPECT_EQ(decideAutoStartAction(State::PRIMARY_STATE_INACTIVE, false, false),
            AutoStartAction::kActivate);
}

TEST(ShotAutoStart, InactiveWhileEstopPressedWaits) {
  EXPECT_EQ(decideAutoStartAction(State::PRIMARY_STATE_INACTIVE, true, false),
            AutoStartAction::kWaitEstopRelease);
}

TEST(ShotAutoStart, InactiveAfterEstopReleaseActivates) {
  EXPECT_EQ(decideAutoStartAction(State::PRIMARY_STATE_INACTIVE, true, true),
            AutoStartAction::kActivate);
}

// ACTIVE 到達後は非常停止状態に関わらずタイマー停止（押下時の解体はエッジ側が行う）
TEST(ShotAutoStart, ActiveStopsTimerRegardlessOfEstop) {
  EXPECT_EQ(decideAutoStartAction(State::PRIMARY_STATE_ACTIVE, false, false),
            AutoStartAction::kStopTimer);
  EXPECT_EQ(decideAutoStartAction(State::PRIMARY_STATE_ACTIVE, true, false),
            AutoStartAction::kStopTimer);
  EXPECT_EQ(decideAutoStartAction(State::PRIMARY_STATE_ACTIVE, true, true),
            AutoStartAction::kStopTimer);
}

TEST(ShotAutoStart, FinalizedStopsTimerAndTransitionStatesDoNothing) {
  EXPECT_EQ(decideAutoStartAction(State::PRIMARY_STATE_FINALIZED, true, true),
            AutoStartAction::kStopTimer);
  EXPECT_EQ(decideAutoStartAction(State::TRANSITION_STATE_CONFIGURING, true, true),
            AutoStartAction::kNone);
  EXPECT_EQ(decideAutoStartAction(State::PRIMARY_STATE_UNKNOWN, true, true),
            AutoStartAction::kNone);
}

TEST(ShotAutoStart, ConfigureFailureRetriesFromUnconfigured) {
  EXPECT_EQ(decideAutoStartAction(State::PRIMARY_STATE_UNCONFIGURED, false, false),
            AutoStartAction::kConfigure);
}

TEST(ShotControllableTimeout, DisabledNeverFailsSafe) {
  EXPECT_FALSE(shouldFailSafeControllableTimeout(0.0, true, true, 10.0));
  EXPECT_FALSE(shouldFailSafeControllableTimeout(-1.0, true, true, 10.0));
}

TEST(ShotControllableTimeout, TrueSignalTimesOutFailSafe) {
  EXPECT_TRUE(shouldFailSafeControllableTimeout(1.0, true, true, 1.01));
}

TEST(ShotControllableTimeout, FalseSignalTimeoutIsNoOp) {
  EXPECT_FALSE(shouldFailSafeControllableTimeout(1.0, true, false, 2.0));
}

TEST(ShotControllableTimeout, NeverReceivedPreservesFallback) {
  EXPECT_FALSE(shouldFailSafeControllableTimeout(1.0, false, false, 100.0));
}

TEST(ShotControllableTimeout, FreshOrRecoveredSignalIsNotStale) {
  EXPECT_FALSE(shouldFailSafeControllableTimeout(1.0, true, true, 0.1));
}

}  // namespace
