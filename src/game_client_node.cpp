#include "rabbits_interface/game_client_node.hpp"

#include <cmath>
#include <functional>

#include <rabcl/controller/omni_drive.hpp>

GameClient::GameClient(const rclcpp::NodeOptions & options)
: Node("game_client_node", options), positions_{}
{
  max_omni_vel_ = this->declare_parameter("max_omni_vel", 0.0);
  max_omni_rot_vel_ = this->declare_parameter("max_omni_rot_vel", 0.0);
  max_yaw_rot_vel_ = this->declare_parameter("max_yaw_rot_vel", 0.0);
  max_pitch_rot_vel_ = this->declare_parameter("max_pitch_rot_vel", 0.0);
  fire_heat_ = this->declare_parameter("fire_heat", 10.0);

  cmd_pub_ = this->create_publisher<rabbits_msgs::msg::AttractsCommand>("cmd", 10);
  joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
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
  positions_.at(4) += static_cast<double>(msg->mouse_delta_x) / 400.0;
  positions_.at(4) = std::fmod(positions_.at(4), 2.0 * M_PI);
  if (positions_.at(4) < 0) {
    positions_.at(4) += 2.0 * M_PI;
  }
  // pitch
  positions_.at(5) += static_cast<double>(msg->mouse_delta_y) / 400.0;
  if (positions_.at(5) < -M_PI / 12) {
    positions_.at(5) = -M_PI / 12;
  }
  if (positions_.at(5) > M_PI / 6) {
    positions_.at(5) = M_PI / 6;
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
  rabbits_msgs::msg::AttractsCommand cmd;  // rad/s
  UpdateCmdVel(cmd);
  UpdatePositions(cmd);
}

void GameClient::UpdateCmdVel(rabbits_msgs::msg::AttractsCommand & cmd)
{
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
  cmd.yaw_pos = positions_.at(4);
  cmd.pitch_pos = positions_.at(5);

  // --- 動作モード
  if (game_data_input_msg_.mouse_right_button) {
    cmd.fire_mode = 1;
    if (game_data_input_msg_.mouse_left_button && CanLoad()) {
      cmd.load_mode = 1;
    }
  }
  if (game_data_input_msg_.key_r) {
    cmd.load_mode = 2;
  }
  cmd.speed_mode = 0;
  if (game_data_input_msg_.key_shift) {
    cmd.chassis_mode = 1;
  }

  cmd.header.stamp = get_clock()->now();
  cmd_pub_->publish(cmd);
}

void GameClient::UpdatePositions(const rabbits_msgs::msg::AttractsCommand & cmd)
{
  rabcl::OmniDrive omni_drive(wheel_d_, body_d_);
  std::array<double, 6> joint_vel;
  omni_drive.CalcVel(
    cmd.chassis_vel.x, cmd.chassis_vel.y, cmd.chassis_vel.z,
    joint_vel.at(0), joint_vel.at(1), joint_vel.at(2), joint_vel.at(3),
    positions_.at(4));

  for (int i = 0; i < 4; i++) {
    positions_.at(i) += joint_vel.at(i) / TIMER_FREQ;
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
  rclcpp::spin(std::make_shared<GameClient>());
  rclcpp::shutdown();
  return 0;
}
#endif
