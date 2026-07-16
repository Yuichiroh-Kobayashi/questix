// Copyright 2026 scramble-robot
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
#ifndef MOTOR_CONTROL_APP__SHOT_COMPONENT_HPP_
#define MOTOR_CONTROL_APP__SHOT_COMPONENT_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "motor_control_lib/servo_control.hpp"

namespace motor_control_app {

// Lifecycle node for the shot (tilt + trigger servo) subsystem.
//
// 非常停止中はサーボバスが通電されず、シリアル接続やサーボ応答が得られない。
// そのため起動時は unconfigured で待機し、通電後に configure（接続 + サーボ応答確認）
// → activate（ホーム移動 + joy 受付開始）で運用状態に遷移する。
// auto_start=true（既定）の場合、内蔵タイマーが configure/activate を成功するまで
// 再試行するので、外部の lifecycle manager なしで systemd 起動に耐える。
class ShotComponent : public rclcpp_lifecycle::LifecycleNode {
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit ShotComponent(const rclcpp::NodeOptions& options);
  ~ShotComponent() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State& state) override;

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void executeShotSequence();
  void autoStartTimerCallback();
  void triggerAutoRecovery();
  void disconnectServo();

  // 角度変換関数
  double clampAngle(double angle_deg);
  bool canSendCommand();
  int angleToServoPosition(double angle_deg);
  double servoPositionToAngle(int position);

  std::shared_ptr<motor_control_lib::FeetechServoController> servo_controller_;

  int tilt_servo_id_;
  int trigger_servo_id_;
  int fire_button_;
  int tilt_axis_;
  int tilt_up_button_index_;
  int tilt_down_button_index_;
  double tilt_step_angle_;
  double tilt_min_angle_;
  double tilt_max_angle_;
  double fire_angle_;
  double home_angle_;
  int fire_duration_ms_;
  int command_rate_limit_ms_;
  bool auto_start_;
  double connect_retry_period_sec_;
  // ACTIVE 中に検出したサーボ通信故障のフラグ。autoStartTimerCallback が拾って
  // deactivate→cleanup→再接続の自動復帰を行う。
  std::atomic<bool> runtime_fault_;

  bool is_shooting_;
  bool last_button_state_;
  float last_tilt_value_;
  bool last_tilt_up_state_;
  bool last_tilt_down_state_;
  int current_tilt_position_;
  double current_tilt_angle_;
  rclcpp::Time last_command_time_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::TimerBase::SharedPtr auto_start_timer_;
};

}  // namespace motor_control_app

#endif  // MOTOR_CONTROL_APP__SHOT_COMPONENT_HPP_
