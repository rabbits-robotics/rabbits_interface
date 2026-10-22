#include "rabbits_interface/game_client_node.hpp"

#include <cmath>
#include <functional>

#include "rabbits_interface/reference_builder.hpp"

using rabbits_interface::MODE_CHASSIS;
using rabbits_interface::MODE_FIRE;
using rabbits_interface::MODE_LOAD;
using rabbits_interface::MODE_SPEED;
using rabbits_interface::PITCH_JOINT;
using rabbits_interface::YAW_JOINT;

GameClient::GameClient(const rclcpp::NodeOptions & options)
: Node("game_client_node", options)
{
  max_omni_vel_ = this->declare_parameter("max_omni_vel", 0.0);
  max_omni_rot_vel_ = this->declare_parameter("max_omni_rot_vel", 0.0);
  max_yaw_rot_vel_ = this->declare_parameter("max_yaw_rot_vel", 0.0);
  max_pitch_rot_vel_ = this->declare_parameter("max_pitch_rot_vel", 0.0);
  fire_heat_ = this->declare_parameter("fire_heat", 10.0);

  cmd_pub_ = this->create_publisher<rabbits_msgs::msg::ReferenceCommand>("reference_command", 10);
  game_data_input_sub_ = this->create_subscription<rabbits_msgs::msg::GameDataInput>(
    "/game_data_input", 10,
    std::bind(&GameClient::GameDataInputCB, this, std::placeholders::_1));
  game_data_robot_sub_ = this->create_subscription<rabbits_msgs::msg::GameDataRobot>(
    "/game_data_robot", 10,
    std::bind(&GameClient::GameDataRobotCB, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / TIMER_FREQ),
    std::bind(&GameClient::TimerCB, this));
}

void GameClient::GameDataInputCB(const rabbits_msgs::msg::GameDataInput::SharedPtr msg)
{
  game_data_input_msg_ = *msg;

  // 砲塔（マウス）はフレーム受信ごとに1回だけ積分する。
  // mouse_delta は相対変位イベントなので、timer で毎ティック積分すると多重に積算される。
  // yaw
  yaw_target_ += static_cast<double>(msg->mouse_delta_x) / 400.0;
  yaw_target_ = std::fmod(yaw_target_, 2.0 * M_PI);
  if (yaw_target_ < 0) {
    yaw_target_ += 2.0 * M_PI;
  }
  // pitch
  pitch_target_ += static_cast<double>(msg->mouse_delta_y) / 400.0;
  if (pitch_target_ < -M_PI / 12) {
    pitch_target_ = -M_PI / 12;
  }
  if (pitch_target_ > M_PI / 6) {
    pitch_target_ = M_PI / 6;
  }
}

void GameClient::GameDataRobotCB(const rabbits_msgs::msg::GameDataRobot::SharedPtr msg)
{
  game_data_robot_msg_ = *msg;
}

bool GameClient::CanLoad() const
{
  // 熱量データ未受信のときはリミットを適用しない。
  if (game_data_robot_msg_.max_heat == 0) {
    return true;
  }
  // 次の射撃で熱量が最大に達する場合は装填を禁止する（オーバーヒート防止）。
  return game_data_robot_msg_.current_heat + fire_heat_ < game_data_robot_msg_.max_heat;
}

void GameClient::TimerCB()
{
  rabbits_msgs::msg::ReferenceCommand cmd;
  UpdateCmdVel(cmd);
}

void GameClient::UpdateCmdVel(rabbits_msgs::msg::ReferenceCommand & cmd)
{
  rabbits_interface::InitReference(cmd);

  // --- 足回り
  // 並進
  if (game_data_input_msg_.key_w) {
    cmd.chassis_vel.x = max_omni_vel_;
  }
  if (game_data_input_msg_.key_s) {
    cmd.chassis_vel.x = -max_omni_vel_;
  }
  if (game_data_input_msg_.key_a) {
    cmd.chassis_vel.y = max_omni_vel_;
  }
  if (game_data_input_msg_.key_d) {
    cmd.chassis_vel.y = -max_omni_vel_;
  }
  // 回転
  if (game_data_input_msg_.key_z) {
    cmd.chassis_vel.z = max_omni_rot_vel_;
  } else if (game_data_input_msg_.key_c) {
    cmd.chassis_vel.z = -1.0 * max_omni_rot_vel_;
  }

  // --- 砲塔（積分は GameDataInputCB 側で実施済み。ここでは保持値を読むだけ）
  rabbits_interface::SetJointPosition(cmd, YAW_JOINT, yaw_target_);
  rabbits_interface::SetJointPosition(cmd, PITCH_JOINT, pitch_target_);

  // --- 動作モード
  uint8_t fire = 0;
  uint8_t load = 0;
  if (game_data_input_msg_.mouse_right_button) {
    fire = 1;
    if (game_data_input_msg_.mouse_left_button && CanLoad()) {
      load = 1;
    }
  }
  if (game_data_input_msg_.key_r) {
    load = 2;
  }
  rabbits_interface::SetMode(cmd, MODE_FIRE, fire);
  rabbits_interface::SetMode(cmd, MODE_LOAD, load);
  rabbits_interface::SetMode(cmd, MODE_SPEED, 0);
  rabbits_interface::SetMode(cmd, MODE_CHASSIS, game_data_input_msg_.key_shift ? 1 : 0);

  cmd.header.stamp = get_clock()->now();
  cmd_pub_->publish(cmd);
}

#ifndef RABBITS_INTERFACE_EXCLUDE_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GameClient>());
  rclcpp::shutdown();
  return 0;
}
#endif
