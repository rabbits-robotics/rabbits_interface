#include "rabbits_interface/gamepad_node.hpp"

#include <cmath>
#include <functional>

#include <rabcl/controller/omni_drive.hpp>

Gamepad::Gamepad(const rclcpp::NodeOptions & options)
: Node("gamepad_node", options), positions_{}
{
  max_omni_vel_ = this->declare_parameter("max_omni_vel", 0.0);
  max_omni_rot_vel_ = this->declare_parameter("max_omni_rot_vel", 0.0);
  max_yaw_rot_vel_ = this->declare_parameter("max_yaw_rot_vel", 0.0);
  max_pitch_rot_vel_ = this->declare_parameter("max_pitch_rot_vel", 0.0);

  cmd_pub_ = this->create_publisher<rabbits_msgs::msg::AttractsCommand>("cmd", 10);
  joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
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
  rabbits_msgs::msg::AttractsCommand cmd;
  if (joy_msg_.axes.size() > 0 && joy_msg_.buttons.size() > 0) {
    UpdateCmdVel(cmd);
    UpdatePositions(cmd);
  }
}

void Gamepad::UpdateCmdVel(rabbits_msgs::msg::AttractsCommand & cmd)
{
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

  // --- 砲塔
  // yaw
  cmd.yaw_pos = positions_.at(4) + joy_msg_.axes.at(3) / 10.0;
  cmd.yaw_pos = std::fmod(cmd.yaw_pos, 2.0 * M_PI);
  if (cmd.yaw_pos < 0) {
    cmd.yaw_pos += 2.0 * M_PI;
  }
  // pitch
  cmd.pitch_pos = positions_.at(5) + joy_msg_.axes.at(4) / 20.0;
  if (cmd.pitch_pos < -M_PI / 12) {
    cmd.pitch_pos = -M_PI / 12;
  }
  if (cmd.pitch_pos > M_PI / 6) {
    cmd.pitch_pos = M_PI / 6;
  }
  // 前回指令値を保存
  positions_.at(4) = cmd.yaw_pos;
  positions_.at(5) = cmd.pitch_pos;

  // --- 動作モード
  cmd.fire_mode = joy_msg_.buttons.at(5);
  cmd.load_mode = joy_msg_.buttons.at(4);
  if (joy_msg_.buttons.at(0) == 1) {
    cmd.load_mode = 2;
  }
  cmd.speed_mode = 0;
  cmd.chassis_mode = joy_msg_.buttons.at(1);
  cmd.header.stamp = get_clock()->now();
  cmd_pub_->publish(cmd);
}

void Gamepad::UpdatePositions(const rabbits_msgs::msg::AttractsCommand & cmd)
{
  rabcl::OmniDrive omni_drive(wheel_d_, body_d_);
  std::array<double, 6> joint_vel;
  omni_drive.CalcVel(
    cmd.chassis_vel.x, cmd.chassis_vel.y, cmd.chassis_vel.z,
    joint_vel.at(0), joint_vel.at(1), joint_vel.at(2), joint_vel.at(3),
    positions_.at(4));
  positions_.at(4) = cmd.yaw_pos;
  positions_.at(5) = cmd.pitch_pos;

  double joy_freq = 100.0;  // Hz
  for (int i = 0; i < 4; i++) {
    positions_.at(i) += joint_vel.at(i) / joy_freq;
  }

  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = get_clock()->now();
  joint_state.name.emplace_back("right_front_wheel_joint");
  joint_state.name.emplace_back("left_front_wheel_joint");
  joint_state.name.emplace_back("right_back_wheel_joint");
  joint_state.name.emplace_back("left_back_wheel_joint");
  joint_state.name.emplace_back("yaw_joint");
  joint_state.name.emplace_back("pitch_joint");
  joint_state.position.emplace_back(positions_.at(0));
  joint_state.position.emplace_back(positions_.at(1));
  joint_state.position.emplace_back(positions_.at(2));
  joint_state.position.emplace_back(positions_.at(3));
  joint_state.position.emplace_back(positions_.at(4));
  joint_state.position.emplace_back(positions_.at(5));
  joint_state_pub_->publish(joint_state);
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
