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

  cmd_pub_ = this->create_publisher<rabbits_msgs::msg::ReferenceCommand>("reference_command", 10);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10,
    std::bind(&Gamepad::JoyCB, this, std::placeholders::_1));
  using namespace std::chrono_literals;
  timer_ = this->create_wall_timer(10ms, std::bind(&Gamepad::TimerCB, this));
}

void Gamepad::JoyCB(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  joy_msg_ = *msg;
}

void Gamepad::TimerCB()
{
  rabbits_msgs::msg::ReferenceCommand cmd;
  if (joy_msg_.axes.size() > 0 && joy_msg_.buttons.size() > 0) {
    UpdateCmdVel(cmd);
  }
}

void Gamepad::UpdateCmdVel(rabbits_msgs::msg::ReferenceCommand & cmd)
{
  rabbits_interface::InitReference(cmd);

  // --- 足回り
  // 並進（ジョイスティック基本、十字キーで上書き）
  cmd.chassis_vel.x = max_omni_vel_ * joy_msg_.axes.at(1);
  cmd.chassis_vel.y = max_omni_vel_ * joy_msg_.axes.at(0);
  if (joy_msg_.axes.at(7) != 0.0) {
    cmd.chassis_vel.x = max_omni_vel_ * joy_msg_.axes.at(7);
  }
  if (joy_msg_.axes.at(6) != 0.0) {
    cmd.chassis_vel.y = max_omni_vel_ * joy_msg_.axes.at(6);
  }
  // 回転
  cmd.chassis_vel.z = 0.0;
  cmd.chassis_vel.z += max_omni_rot_vel_ * 0.5 * (1.0 - joy_msg_.axes.at(5));
  cmd.chassis_vel.z += -1.0 * max_omni_rot_vel_ * 0.5 * (1.0 - joy_msg_.axes.at(2));

  // --- 砲塔（前回の目標にスティック分を積分）
  yaw_target_ += joy_msg_.axes.at(3) / 10.0;
  yaw_target_ = std::fmod(yaw_target_, 2.0 * M_PI);
  if (yaw_target_ < 0) {
    yaw_target_ += 2.0 * M_PI;
  }
  pitch_target_ += joy_msg_.axes.at(4) / 20.0;
  if (pitch_target_ < -M_PI / 12) {
    pitch_target_ = -M_PI / 12;
  }
  if (pitch_target_ > M_PI / 6) {
    pitch_target_ = M_PI / 6;
  }
  rabbits_interface::SetJointPosition(cmd, YAW_JOINT, yaw_target_);
  rabbits_interface::SetJointPosition(cmd, PITCH_JOINT, pitch_target_);

  // --- 動作モード
  rabbits_interface::SetMode(cmd, MODE_FIRE, static_cast<uint8_t>(joy_msg_.buttons.at(5)));
  uint8_t load = static_cast<uint8_t>(joy_msg_.buttons.at(4));
  if (joy_msg_.buttons.at(0) == 1) {
    load = 2;
  }
  rabbits_interface::SetMode(cmd, MODE_LOAD, load);
  rabbits_interface::SetMode(cmd, MODE_SPEED, 0);
  rabbits_interface::SetMode(cmd, MODE_CHASSIS, static_cast<uint8_t>(joy_msg_.buttons.at(1)));

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
