#ifndef RABBITS_INTERFACE__GAMEPAD_NODE_HPP_
#define RABBITS_INTERFACE__GAMEPAD_NODE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/empty.hpp>

#include <rabbits_msgs/msg/reference_command.hpp>

#include <rabcl/controller/view_target.hpp>

// ゲームパッド（sensor_msgs/Joy、joy_linux の DualShock 4 配列）を ReferenceCommand に変換して
// reference_command に出す。デバッグ用。割り当て:
//   十字キー（axes 6 / 7）/ 左スティック（axes 0 / 1）並進、L2 / R2（axes 2 / 5）旋回、
//   右スティック（axes 3 / 4）視点、R1 射出、L1 装填、○ 逆転、× 無限回転、△ 歩き、□ しゃがみ、
//   リーン左 / 右は lean_left_button / lean_right_button（背面ボタンの番号。-1 = 無し）
class Gamepad : public rclcpp::Node
{
public:
  explicit Gamepad(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void JoyCB(const sensor_msgs::msg::Joy::SharedPtr msg);
  // ファームの再起動（bridge の firmware_reboot）: 視点の目標を 0 に戻す
  void FirmwareRebootCB(const std_msgs::msg::Empty::SharedPtr msg);
  void TimerCB();
  // joy_msg_ から指令を組み立てて cmd に入れ、配信する
  void UpdateCmdVel(rabbits_msgs::msg::ReferenceCommand & cmd);

private:
  // joy_linux の DualShock 4 配列
  static constexpr size_t AXIS_LEFT_X = 0;
  static constexpr size_t AXIS_LEFT_Y = 1;
  static constexpr size_t AXIS_L2 = 2;
  static constexpr size_t AXIS_RIGHT_X = 3;
  static constexpr size_t AXIS_RIGHT_Y = 4;
  static constexpr size_t AXIS_R2 = 5;
  static constexpr size_t AXIS_DPAD_X = 6;
  static constexpr size_t AXIS_DPAD_Y = 7;
  static constexpr size_t BUTTON_CROSS = 0;
  static constexpr size_t BUTTON_CIRCLE = 1;
  static constexpr size_t BUTTON_TRIANGLE = 2;
  static constexpr size_t BUTTON_SQUARE = 3;
  static constexpr size_t BUTTON_L1 = 4;
  static constexpr size_t BUTTON_R1 = 5;
  static constexpr size_t MIN_AXES = 8;
  static constexpr size_t MIN_BUTTONS = 6;
  static constexpr double YAW_RAD_PER_UNIT = 1.0 / 10.0;    // 右スティック X 1.0 で 1 周期あたり
  static constexpr double PITCH_RAD_PER_UNIT = 1.0 / 20.0;  // 右スティック Y

  bool Pressed(int button) const;

  rclcpp::Publisher<rabbits_msgs::msg::ReferenceCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr firmware_reboot_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double max_omni_vel_;      // m/s
  double max_omni_rot_vel_;  // rad/s
  double max_yaw_rot_vel_;   // rad/s
  double max_pitch_rot_vel_;  // rad/s
  double walk_scale_;  // 歩き（△）のときの速度倍率
  double body_height_default_;  // 通常時の車高（0〜1）
  double lean_roll_;  // リーンのロール [rad]
  int lean_left_button_;   // -1 = 無し
  int lean_right_button_;  // -1 = 無し

  sensor_msgs::msg::Joy joy_msg_;
  rabcl::ViewTarget view_;  // 視点の目標（右スティックで積分）
};

#endif  // RABBITS_INTERFACE__GAMEPAD_NODE_HPP_
