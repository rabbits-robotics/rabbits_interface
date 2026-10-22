#ifndef RABBITS_INTERFACE__GAME_CLIENT_NODE_HPP_
#define RABBITS_INTERFACE__GAME_CLIENT_NODE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <rabbits_msgs/msg/game_data_input.hpp>
#include <rabbits_msgs/msg/game_data_robot.hpp>
#include <rabbits_msgs/msg/reference_command.hpp>

// ゲーム側の入力（GameDataInput）を ReferenceCommand に変換して reference_command に出す
class GameClient : public rclcpp::Node
{
public:
  explicit GameClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void GameDataInputCB(const rabbits_msgs::msg::GameDataInput::SharedPtr msg);
  void GameDataRobotCB(const rabbits_msgs::msg::GameDataRobot::SharedPtr msg);
  void TimerCB();
  // 保持している入力から指令を組み立てて cmd に入れ、配信する
  void UpdateCmdVel(rabbits_msgs::msg::ReferenceCommand & cmd);

  // 次の射撃で熱量が最大に達する場合は装填を禁止する（オーバーヒート防止）。
  // 熱量データ未受信（max_heat == 0）のときはリミットを適用しない。
  bool CanLoad() const;

private:
  static constexpr double TIMER_FREQ = 100.0;  // Hz

  rclcpp::Publisher<rabbits_msgs::msg::ReferenceCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<rabbits_msgs::msg::GameDataInput>::SharedPtr game_data_input_sub_;
  rclcpp::Subscription<rabbits_msgs::msg::GameDataRobot>::SharedPtr game_data_robot_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double max_omni_vel_;     // m/s
  double max_omni_rot_vel_;  // rad/s
  double max_yaw_rot_vel_;   // rad/s
  double max_pitch_rot_vel_;  // rad/s
  double fire_heat_;  // 1射あたりの熱量増加量

  rabbits_msgs::msg::GameDataInput game_data_input_msg_;
  rabbits_msgs::msg::GameDataRobot game_data_robot_msg_;
  double yaw_target_ = 0.0;    // rad（マウスの相対移動を受信ごとに積分）
  double pitch_target_ = 0.0;  // rad
};

#endif  // RABBITS_INTERFACE__GAME_CLIENT_NODE_HPP_
