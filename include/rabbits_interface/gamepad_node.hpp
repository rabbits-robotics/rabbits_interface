#ifndef RABBITS_INTERFACE__GAMEPAD_NODE_HPP_
#define RABBITS_INTERFACE__GAMEPAD_NODE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <rabbits_msgs/msg/reference_command.hpp>

// ゲームパッド（sensor_msgs/Joy）を ReferenceCommand に変換して reference_command に出す
class Gamepad : public rclcpp::Node
{
public:
  explicit Gamepad(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void JoyCB(const sensor_msgs::msg::Joy::SharedPtr msg);
  void TimerCB();
  // joy_msg_ から指令を組み立てて cmd に入れ、配信する
  void UpdateCmdVel(rabbits_msgs::msg::ReferenceCommand & cmd);

private:
  rclcpp::Publisher<rabbits_msgs::msg::ReferenceCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double max_omni_vel_;     // m/s
  double max_omni_rot_vel_;  // rad/s
  double max_yaw_rot_vel_;   // rad/s
  double max_pitch_rot_vel_;  // rad/s

  sensor_msgs::msg::Joy joy_msg_;
  double yaw_target_ = 0.0;    // rad（スティックで積分）
  double pitch_target_ = 0.0;  // rad
};

#endif  // RABBITS_INTERFACE__GAMEPAD_NODE_HPP_
