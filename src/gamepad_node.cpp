#include "rabbits_interface/gamepad_node.hpp"

#include <cmath>
#include <functional>

#include "rabbits_interface/reference_builder.hpp"

using rabbits_interface::MODE_CHASSIS;
using rabbits_interface::MODE_FIRE;
using rabbits_interface::MODE_LOAD;
using rabbits_interface::MODE_SPEED;
using rabbits_interface::PITCH_JOINT;
using rabbits_interface::YAW_JOINT;

Gamepad::Gamepad(const rclcpp::NodeOptions & options)
: Node("gamepad_node", options)
{
  max_omni_vel_ = this->declare_parameter("max_omni_vel", 0.0);
  max_omni_rot_vel_ = this->declare_parameter("max_omni_rot_vel", 0.0);
  max_yaw_rot_vel_ = this->declare_parameter("max_yaw_rot_vel", 0.0);
  max_pitch_rot_vel_ = this->declare_parameter("max_pitch_rot_vel", 0.0);
  walk_scale_ = this->declare_parameter("walk_scale", 0.4);
  body_height_default_ = this->declare_parameter("body_height_default", 0.5);
  lean_roll_ = this->declare_parameter("lean_roll", 0.15);
  lean_left_button_ = static_cast<int>(this->declare_parameter<int64_t>("lean_left_button", -1));
  lean_right_button_ =
    static_cast<int>(this->declare_parameter<int64_t>("lean_right_button", -1));

  view_.SetPitchLimits(-static_cast<float>(M_PI) / 12.0f, static_cast<float>(M_PI) / 6.0f);

  cmd_pub_ = this->create_publisher<rabbits_msgs::msg::ReferenceCommand>("reference_command", 10);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10,
    std::bind(&Gamepad::JoyCB, this, std::placeholders::_1));
  firmware_reboot_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "firmware_reboot", 10,
    std::bind(&Gamepad::FirmwareRebootCB, this, std::placeholders::_1));
  using namespace std::chrono_literals;
  timer_ = this->create_wall_timer(10ms, std::bind(&Gamepad::TimerCB, this));
}

void Gamepad::JoyCB(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  joy_msg_ = *msg;
}

void Gamepad::FirmwareRebootCB(const std_msgs::msg::Empty::SharedPtr)
{
  RCLCPP_INFO(this->get_logger(), "firmware rebooted: resetting view target");
  view_.Reset({0.0f, 0.0f, 0.0f});
}

void Gamepad::TimerCB()
{
  rabbits_msgs::msg::ReferenceCommand cmd;
  if (joy_msg_.axes.size() >= MIN_AXES && joy_msg_.buttons.size() >= MIN_BUTTONS) {
    UpdateCmdVel(cmd);
  }
}

bool Gamepad::Pressed(int button) const
{
  return button >= 0 && static_cast<size_t>(button) < joy_msg_.buttons.size() &&
         joy_msg_.buttons[static_cast<size_t>(button)] != 0;
}

void Gamepad::UpdateCmdVel(rabbits_msgs::msg::ReferenceCommand & cmd)
{
  const auto & axes = joy_msg_.axes;
  rabbits_interface::InitReference(cmd, body_height_default_);

  // --- 足回り（△ で歩き: 速度を落として speed = 0）
  const bool walk = Pressed(BUTTON_TRIANGLE);
  const double vel = max_omni_vel_ * (walk ? walk_scale_ : 1.0);
  const double rot = max_omni_rot_vel_ * (walk ? walk_scale_ : 1.0);
  // 並進（左スティック基本、十字キーで上書き）
  cmd.chassis_vel.x = vel * axes.at(AXIS_LEFT_Y);
  cmd.chassis_vel.y = vel * axes.at(AXIS_LEFT_X);
  if (axes.at(AXIS_DPAD_Y) != 0.0) {
    cmd.chassis_vel.x = vel * axes.at(AXIS_DPAD_Y);
  }
  if (axes.at(AXIS_DPAD_X) != 0.0) {
    cmd.chassis_vel.y = vel * axes.at(AXIS_DPAD_X);
  }
  // 旋回（トリガーは離して +1、全押しで -1）
  cmd.chassis_vel.z = rot * 0.5 * (1.0 - axes.at(AXIS_R2)) - rot * 0.5 * (1.0 - axes.at(AXIS_L2));

  // --- 車体姿勢（□ でしゃがみ、背面ボタンでリーン）
  if (Pressed(BUTTON_SQUARE)) {
    cmd.body_height = 0.0f;
  }
  if (Pressed(lean_left_button_)) {
    cmd.body_roll = static_cast<float>(-lean_roll_);
  } else if (Pressed(lean_right_button_)) {
    cmd.body_roll = static_cast<float>(lean_roll_);
  }

  // --- 視点（右スティック分を積分）
  view_.AddDelta(
    static_cast<float>(axes.at(AXIS_RIGHT_X) * YAW_RAD_PER_UNIT),
    static_cast<float>(axes.at(AXIS_RIGHT_Y) * PITCH_RAD_PER_UNIT));
  const rabcl::Attitude view = view_.Get();
  rabbits_interface::SetJointPosition(cmd, YAW_JOINT, view.yaw);
  rabbits_interface::SetJointPosition(cmd, PITCH_JOINT, view.pitch);

  // --- 動作モード
  rabbits_interface::SetMode(cmd, MODE_FIRE, Pressed(BUTTON_R1) ? 1 : 0);
  uint8_t load = Pressed(BUTTON_L1) ? 1 : 0;
  if (Pressed(BUTTON_CIRCLE)) {
    load = 2;
  }
  rabbits_interface::SetMode(cmd, MODE_LOAD, load);
  rabbits_interface::SetMode(cmd, MODE_SPEED, walk ? 0 : 1);
  rabbits_interface::SetMode(cmd, MODE_CHASSIS, Pressed(BUTTON_CROSS) ? 1 : 0);

  cmd.header.stamp = get_clock()->now();
  cmd_pub_->publish(cmd);
}

#ifndef RABBITS_INTERFACE_EXCLUDE_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Gamepad>());
  rclcpp::shutdown();
  return 0;
}
#endif
