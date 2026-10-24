#ifndef RABBITS_INTERFACE__GAME_CLIENT_NODE_HPP_
#define RABBITS_INTERFACE__GAME_CLIENT_NODE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include <rabbits_msgs/msg/game_data_input.hpp>
#include <rabbits_msgs/msg/game_data_robot.hpp>
#include <rabbits_msgs/msg/reference_command.hpp>

#include <rabcl/controller/view_target.hpp>

// ゲーム側の入力（GameDataInput）を ReferenceCommand に変換して reference_command に出す。
// 割り当て（robot_ws/gamma README「操作と制御の仕様」と同じ）:
//   W/A/S/D 並進、Z/C 旋回、マウス 視点、右クリック 射出、左クリック 装填、R 逆転、
//   Ctrl 歩き（押している間、walk_scale 倍 + speed 0）、Shift しゃがみ（body_height 0）、
//   Space 無限回転（chassis 1）、Q/E リーン（body_roll ∓ lean_roll）。stabilize は常に 1
class GameClient : public rclcpp::Node
{
public:
  explicit GameClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void GameDataInputCB(const rabbits_msgs::msg::GameDataInput::SharedPtr msg);
  void GameDataRobotCB(const rabbits_msgs::msg::GameDataRobot::SharedPtr msg);
  // ファームの再起動（bridge の firmware_reboot）: 視点の目標を 0 に戻す（ファーム側の基準も起動時の向き）
  void FirmwareRebootCB(const std_msgs::msg::Empty::SharedPtr msg);
  void TimerCB();
  // 保持している入力から指令を組み立てて cmd に入れ、配信する
  void UpdateCmdVel(rabbits_msgs::msg::ReferenceCommand & cmd);

  // 次の射撃で熱量が最大に達する場合は装填を禁止する（オーバーヒート防止）。
  // 熱量データ未受信（max_heat == 0）のときはリミットを適用しない。
  bool CanLoad() const;

private:
  static constexpr double TIMER_FREQ = 100.0;  // Hz
  static constexpr double MOUSE_COUNTS_PER_RAD = 400.0;

  rclcpp::Publisher<rabbits_msgs::msg::ReferenceCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<rabbits_msgs::msg::GameDataInput>::SharedPtr game_data_input_sub_;
  rclcpp::Subscription<rabbits_msgs::msg::GameDataRobot>::SharedPtr game_data_robot_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr firmware_reboot_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double max_omni_vel_;      // m/s
  double max_omni_rot_vel_;  // rad/s
  double max_yaw_rot_vel_;   // rad/s
  double max_pitch_rot_vel_;  // rad/s
  double fire_heat_;  // 1射あたりの熱量増加量
  double walk_scale_;  // 歩き（Ctrl）のときの速度倍率
  double body_height_default_;  // 通常時の車高（0〜1）
  double lean_roll_;  // リーン（Q/E）のロール [rad]

  rabbits_msgs::msg::GameDataInput game_data_input_msg_;
  rabbits_msgs::msg::GameDataRobot game_data_robot_msg_;
  rabcl::ViewTarget view_;  // 視点の目標（マウスの相対移動を受信ごとに積分）
};

#endif  // RABBITS_INTERFACE__GAME_CLIENT_NODE_HPP_
