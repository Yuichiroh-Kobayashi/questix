// Copyright 2026 scramble-robot
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
#include "motor_control_app/shot_component.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <lifecycle_msgs/msg/state.hpp>
#include <string>
#include <thread>

namespace motor_control_app {

ShotComponent::ShotComponent(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("shot_component", options),
      tilt_servo_id_(1),
      trigger_servo_id_(3),
      fire_button_(5),
      tilt_axis_(-1),
      tilt_up_button_index_(4),
      tilt_down_button_index_(6),
      tilt_step_angle_(5.0),
      tilt_min_angle_(0.0),
      tilt_max_angle_(70.0),
      fire_angle_(130.0),
      home_angle_(100.0),
      fire_duration_ms_(300),
      command_rate_limit_ms_(50),
      auto_start_(true),
      connect_retry_period_sec_(3.0),
      is_shooting_(false),
      last_button_state_(false),
      last_tilt_value_(0.0F),
      last_tilt_up_state_(false),
      last_tilt_down_state_(false),
      current_tilt_position_(2048),
      current_tilt_angle_(0.0),
      last_command_time_(0, 0, RCL_ROS_TIME) {
  // パラメーター宣言（取得は on_configure で行い、cleanup→configure で再読込できるようにする）
  this->declare_parameter("port", "/dev/servo");
  this->declare_parameter("baudrate", 115200);
  this->declare_parameter("tilt_servo_id", 1);
  this->declare_parameter("trigger_servo_id", 3);
  this->declare_parameter("fire_button", 5);  // R button (Switch2 native index)
  // Tilt input mode / チルト入力モード
  // - If tilt_axis >= 0: use the analog axis (D-pad / stick).
  // - Otherwise: use tilt_up_button_index / tilt_down_button_index (button edge).
  // tilt_axis が 0 以上なら軸モード、それ以外はボタンモードで動いて上下します。
  this->declare_parameter("tilt_axis", -1);              // -1 = disabled (button mode)
  this->declare_parameter("tilt_up_button_index", 4);    // L button (Switch2 native index)
  this->declare_parameter("tilt_down_button_index", 6);  // ZL button (Switch2 native index)
  this->declare_parameter("tilt_step_angle", 5.0);       // チルトステップサイズ（度）
  this->declare_parameter("tilt_min_angle", 0.0);        // チルト最小角度（度）
  this->declare_parameter("tilt_max_angle", 70.0);       // チルト最大角度（度）
  this->declare_parameter("fire_angle", 130.0);          // 射撃角度（度）
  this->declare_parameter("home_angle", 100.0);          // ホーム角度（度）
  this->declare_parameter("fire_duration_ms", 300);      // 射撃持続時間（ミリ秒）
  this->declare_parameter("command_rate_limit_ms", 50);  // コマンド間隔制限（ミリ秒）
  this->declare_parameter("joy_topic", "/joy");          // joyトピック名
  // Lifecycle 自動遷移。非常停止解除でサーボが通電するまで configure を再試行する。
  this->declare_parameter("auto_start", true);
  this->declare_parameter("connect_retry_period_sec", 3.0);

  auto_start_ = this->get_parameter("auto_start").as_bool();
  connect_retry_period_sec_ = this->get_parameter("connect_retry_period_sec").as_double();

  if (auto_start_) {
    const auto period = std::chrono::duration<double>(std::max(0.5, connect_retry_period_sec_));
    auto_start_timer_ =
        this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                std::bind(&ShotComponent::autoStartTimerCallback, this));
    RCLCPP_INFO(this->get_logger(),
                "Shot component created (auto_start=true, retry=%.1fs). "
                "サーボ通電（非常停止解除）を待って自動起動します",
                connect_retry_period_sec_);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Shot component created (auto_start=false). "
                "外部から lifecycle configure/activate してください");
  }
}

ShotComponent::~ShotComponent() {
  if (auto_start_timer_) {
    auto_start_timer_->cancel();
  }
  disconnectServo();
}

void ShotComponent::autoStartTimerCallback() {
  const uint8_t state_id = this->get_current_state().id();
  if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                         "サーボ接続を試行します（非常停止中は失敗し、解除後に自動復帰します）");
    this->configure();
  } else if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    this->activate();
  }
  // ACTIVE の間は何もしない。エラーで unconfigured に戻れば次周期で再試行する。
}

ShotComponent::CallbackReturn ShotComponent::on_configure(const rclcpp_lifecycle::State&) {
  std::string port = this->get_parameter("port").as_string();
  int baudrate = this->get_parameter("baudrate").as_int();
  tilt_servo_id_ = this->get_parameter("tilt_servo_id").as_int();
  trigger_servo_id_ = this->get_parameter("trigger_servo_id").as_int();
  fire_button_ = this->get_parameter("fire_button").as_int();
  tilt_axis_ = this->get_parameter("tilt_axis").as_int();
  tilt_up_button_index_ = this->get_parameter("tilt_up_button_index").as_int();
  tilt_down_button_index_ = this->get_parameter("tilt_down_button_index").as_int();
  tilt_step_angle_ = this->get_parameter("tilt_step_angle").as_double();
  tilt_min_angle_ = this->get_parameter("tilt_min_angle").as_double();
  tilt_max_angle_ = this->get_parameter("tilt_max_angle").as_double();
  fire_angle_ = this->get_parameter("fire_angle").as_double();
  home_angle_ = this->get_parameter("home_angle").as_double();
  fire_duration_ms_ = this->get_parameter("fire_duration_ms").as_int();
  command_rate_limit_ms_ = this->get_parameter("command_rate_limit_ms").as_int();
  std::string joy_topic = this->get_parameter("joy_topic").as_string();

  // サーボコントローラー接続（非常停止中はポートが無い / 開けない場合がある）
  servo_controller_ = std::make_shared<motor_control_lib::FeetechServoController>(port, baudrate);
  if (!servo_controller_->connect()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                         "サーボ接続に失敗しました (port=%s)。通電を待って再試行します",
                         port.c_str());
    servo_controller_.reset();
    return CallbackReturn::FAILURE;
  }

  // ポートが開けても未通電ならサーボは応答しないため、実際に1レジスタ読んで確認する
  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 初期化待機
  int32_t current_pos = servo_controller_->getCurrentPosition(tilt_servo_id_);
  if (current_pos == -1) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                         "サーボ %d が応答しません（非常停止による未通電の可能性）。再試行します",
                         tilt_servo_id_);
    disconnectServo();
    return CallbackReturn::FAILURE;
  }
  current_tilt_position_ = current_pos;
  current_tilt_angle_ = clampAngle(servoPositionToAngle(current_pos));

  // joyサブスクライバー作成（コールバックは ACTIVE のときのみ処理する）
  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic, 1, std::bind(&ShotComponent::joyCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Shot component configured (current tilt: %.1f deg)",
              current_tilt_angle_);
  return CallbackReturn::SUCCESS;
}

ShotComponent::CallbackReturn ShotComponent::on_activate(const rclcpp_lifecycle::State&) {
  if (!servo_controller_ || !servo_controller_->isConnected()) {
    RCLCPP_ERROR(this->get_logger(), "Servo controller not connected, cannot activate");
    return CallbackReturn::FAILURE;
  }

  // ホーム位置に移動
  int home_position = angleToServoPosition(home_angle_);
  if (!servo_controller_->setPosition(trigger_servo_id_, home_position, false)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to move to initial home position（通電断の可能性）。再初期化します");
    return CallbackReturn::FAILURE;
  }

  // エッジ検出状態をリセット（inactive 中に押されたボタンで誤発射しないため）
  is_shooting_ = false;
  last_button_state_ = false;
  last_tilt_value_ = 0.0F;
  last_tilt_up_state_ = false;
  last_tilt_down_state_ = false;
  last_command_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "Shot component activated");
  if (tilt_axis_ >= 0) {
    RCLCPP_INFO(this->get_logger(), "Fire button: %d, Tilt mode: axis=%d, Tilt step: %.1f degrees",
                fire_button_, tilt_axis_, tilt_step_angle_);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Fire button: %d, Tilt mode: buttons up=%d down=%d, Tilt step: %.1f degrees",
                fire_button_, tilt_up_button_index_, tilt_down_button_index_, tilt_step_angle_);
  }
  RCLCPP_INFO(this->get_logger(), "Tilt range: %.1f - %.1f degrees", tilt_min_angle_,
              tilt_max_angle_);
  RCLCPP_INFO(this->get_logger(),
              "Fire angle: %.1f deg, Home angle: %.1f deg, Current tilt: %.1f deg", fire_angle_,
              home_angle_, current_tilt_angle_);
  if (auto_start_timer_) {
    auto_start_timer_->cancel();
  }
  return CallbackReturn::SUCCESS;
}

ShotComponent::CallbackReturn ShotComponent::on_deactivate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(this->get_logger(), "Shot component deactivated (joy input ignored)");
  return CallbackReturn::SUCCESS;
}

ShotComponent::CallbackReturn ShotComponent::on_cleanup(const rclcpp_lifecycle::State&) {
  joy_subscription_.reset();
  disconnectServo();
  RCLCPP_INFO(this->get_logger(), "Shot component cleaned up");
  return CallbackReturn::SUCCESS;
}

ShotComponent::CallbackReturn ShotComponent::on_shutdown(const rclcpp_lifecycle::State&) {
  joy_subscription_.reset();
  disconnectServo();
  RCLCPP_INFO(this->get_logger(), "Shot component shut down");
  return CallbackReturn::SUCCESS;
}

ShotComponent::CallbackReturn ShotComponent::on_error(const rclcpp_lifecycle::State&) {
  // 遷移中の失敗（activate 時のホーム移動失敗など）はリソースを解放して
  // unconfigured に戻し、auto_start タイマーの再試行に委ねる。
  joy_subscription_.reset();
  disconnectServo();
  if (auto_start_ && auto_start_timer_) {
    auto_start_timer_->reset();
  }
  RCLCPP_WARN(this->get_logger(), "Shot component error handled, returning to unconfigured");
  return CallbackReturn::SUCCESS;
}

void ShotComponent::disconnectServo() {
  if (servo_controller_) {
    servo_controller_->disconnect();
    servo_controller_.reset();
  }
}

void ShotComponent::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }
  if (!servo_controller_ || !servo_controller_->isConnected()) {
    return;
  }
  if (!msg || msg->buttons.empty()) {
    return;
  }

  // 射撃ボタンの処理
  if (!is_shooting_ && fire_button_ >= 0 && fire_button_ < static_cast<int>(msg->buttons.size())) {
    bool current_button_state = msg->buttons[fire_button_] == 1;

    // ボタンが押された瞬間を検出（立ち上がりエッジ）
    if (current_button_state && !last_button_state_) {
      executeShotSequence();
    }

    last_button_state_ = current_button_state;
  }

  // axesが存在するかチェック
  if (msg->axes.empty()) {
    return;
  }

  // Tilt input: axis mode if tilt_axis >= 0, otherwise button mode (L / ZL).
  // tilt_axis が 0 以上なら軸モード、それ以外はボタンモード。
  const auto step_tilt = [this](double delta_deg, const char* direction) {
    if (!canSendCommand()) {
      RCLCPP_DEBUG(this->get_logger(), "Tilt command rate limited");
      return;
    }
    double new_angle = current_tilt_angle_ + delta_deg;
    current_tilt_angle_ = clampAngle(new_angle);
    current_tilt_position_ = angleToServoPosition(current_tilt_angle_);
    if (servo_controller_->setPosition(tilt_servo_id_, current_tilt_position_, false)) {
      RCLCPP_INFO(this->get_logger(), "Tilt %s: angle=%.1f deg", direction, current_tilt_angle_);
      last_command_time_ = this->now();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to move tilt %s", direction);
    }
  };

  if (tilt_axis_ >= 0 && tilt_axis_ < static_cast<int>(msg->axes.size())) {
    // Axis mode: rising edge detection at ±0.5
    const float current_tilt_value = msg->axes[tilt_axis_];
    if (current_tilt_value > 0.5F && last_tilt_value_ <= 0.5F) {
      step_tilt(tilt_step_angle_, "up");
    } else if (current_tilt_value < -0.5F && last_tilt_value_ >= -0.5F) {
      step_tilt(-tilt_step_angle_, "down");
    }
    last_tilt_value_ = current_tilt_value;
  } else {
    // Button mode: L = up, ZL = down (rising edge)
    const auto button_pressed = [&msg](int index) {
      return index >= 0 && static_cast<size_t>(index) < msg->buttons.size() &&
             msg->buttons[static_cast<size_t>(index)] == 1;
    };
    const bool tilt_up_pressed = button_pressed(tilt_up_button_index_);
    const bool tilt_down_pressed = button_pressed(tilt_down_button_index_);

    if (tilt_up_pressed && !last_tilt_up_state_) {
      step_tilt(tilt_step_angle_, "up");
    } else if (tilt_down_pressed && !last_tilt_down_state_) {
      step_tilt(-tilt_step_angle_, "down");
    }
    last_tilt_up_state_ = tilt_up_pressed;
    last_tilt_down_state_ = tilt_down_pressed;
  }
}

void ShotComponent::executeShotSequence() {
  if (is_shooting_) {
    return;  // 既に射撃中の場合は無視
  }

  is_shooting_ = true;
  RCLCPP_INFO(this->get_logger(), "Starting shot sequence...");

  // 1. 射撃位置に移動
  int fire_position = angleToServoPosition(fire_angle_);
  if (servo_controller_->setPosition(trigger_servo_id_, fire_position, false)) {
    RCLCPP_INFO(this->get_logger(), "Moved to fire position (%.1f deg)", fire_angle_);

    // 射撃持続時間待機
    std::this_thread::sleep_for(std::chrono::milliseconds(fire_duration_ms_));

    // 2. すべてのサーボをホーム位置に戻る
    int home_position = angleToServoPosition(home_angle_);
    if (servo_controller_->setPosition(trigger_servo_id_, home_position, false)) {
      RCLCPP_INFO(this->get_logger(), "Returned to home position (%.1f deg)", home_angle_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to return to home position");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to fire position");
  }

  is_shooting_ = false;
  RCLCPP_INFO(this->get_logger(), "Shot sequence completed");
}

// 角度制限関数
double ShotComponent::clampAngle(double angle_deg) {
  return std::max(tilt_min_angle_, std::min(tilt_max_angle_, angle_deg));
}

// コマンド送信レート制限チェック
bool ShotComponent::canSendCommand() {
  auto now = this->now();
  auto elapsed = (now - last_command_time_).nanoseconds() / 1000000;  // ミリ秒に変換
  return elapsed >= command_rate_limit_ms_;
}

// 角度からサーボ位置への変換（角度 -> 0-4095）
int ShotComponent::angleToServoPosition(double angle_deg) {
  // 角度を直接サーボ位置に変換（0度=0, 360度=4095）
  // 角度を0-360度の範囲で正規化
  while (angle_deg < 0) angle_deg += 360.0;
  while (angle_deg >= 360.0) angle_deg -= 360.0;

  // サーボ位置に変換
  double normalized = angle_deg / 360.0;
  int position = static_cast<int>(normalized * 4096.0);
  return std::max(0, std::min(4095, position));
}

// サーボ位置から角度への変換（0-4095 -> 角度）
double ShotComponent::servoPositionToAngle(int position) {
  // 位置を0-4095の範囲にクランプ
  position = std::max(0, std::min(4095, position));
  // 角度に変換（0-4095 -> 0-360度）
  double normalized = static_cast<double>(position) / 4096.0;
  return normalized * 360.0;
}

}  // namespace motor_control_app

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(motor_control_app::ShotComponent)
