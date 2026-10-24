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
  walk_scale_ = this->declare_parameter("walk_scale", 0.4);
  body_height_default_ = this->declare_parameter("body_height_default", 0.5);
  lean_roll_ = this->declare_parameter("lean_roll", 0.15);

  view_.SetPitchLimits(-static_cast<float>(M_PI) / 12.0f, static_cast<float>(M_PI) / 6.0f);

  cmd_pub_ = this->create_publisher<rabbits_msgs::msg::ReferenceCommand>("reference_command", 10);
  game_data_input_sub_ = this->create_subscription<rabbits_msgs::msg::GameDataInput>(
    "/game_data_input", 10,
    std::bind(&GameClient::GameDataInputCB, this, std::placeholders::_1));
  game_data_robot_sub_ = this->create_subscription<rabbits_msgs::msg::GameDataRobot>(
    "/game_data_robot", 10,
    std::bind(&GameClient::GameDataRobotCB, this, std::placeholders::_1));
  firmware_reboot_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "firmware_reboot", 10,
    std::bind(&GameClient::FirmwareRebootCB, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / TIMER_FREQ),
    std::bind(&GameClient::TimerCB, this));
}

void GameClient::GameDataInputCB(const rabbits_msgs::msg::GameDataInput::SharedPtr msg)
{
  game_data_input_msg_ = *msg;

  // 視点（マウス）はフレーム受信ごとに1回だけ積分する。
  // mouse_delta は相対変位イベントなので、timer で毎ティック積分すると多重に積算される。
  view_.AddDelta(
    static_cast<float>(msg->mouse_delta_x / MOUSE_COUNTS_PER_RAD),
    static_cast<float>(msg->mouse_delta_y / MOUSE_COUNTS_PER_RAD));
}

void GameClient::GameDataRobotCB(const rabbits_msgs::msg::GameDataRobot::SharedPtr msg)
{
  game_data_robot_msg_ = *msg;
}

void GameClient::FirmwareRebootCB(const std_msgs::msg::Empty::SharedPtr)
{
  RCLCPP_INFO(this->get_logger(), "firmware rebooted: resetting view target");
  view_.Reset({0.0f, 0.0f, 0.0f});
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
  const auto & in = game_data_input_msg_;
  rabbits_interface::InitReference(cmd, body_height_default_);

  // --- 足回り（Ctrl で歩き: 速度を落として speed = 0）
  const bool walk = in.key_ctrl;
  const double vel = max_omni_vel_ * (walk ? walk_scale_ : 1.0);
  const double rot = max_omni_rot_vel_ * (walk ? walk_scale_ : 1.0);
  if (in.key_w) {
    cmd.chassis_vel.x = vel;
  }
  if (in.key_s) {
    cmd.chassis_vel.x = -vel;
  }
  if (in.key_a) {
    cmd.chassis_vel.y = vel;
  }
  if (in.key_d) {
    cmd.chassis_vel.y = -vel;
  }
  if (in.key_z) {
    cmd.chassis_vel.z = rot;
  } else if (in.key_c) {
    cmd.chassis_vel.z = -rot;
  }

  // --- 車体姿勢（Shift でしゃがみ、Q/E でリーン）
  if (in.key_shift) {
    cmd.body_height = 0.0f;
  }
  if (in.key_q) {
    cmd.body_roll = static_cast<float>(-lean_roll_);
  } else if (in.key_e) {
    cmd.body_roll = static_cast<float>(lean_roll_);
  }

  // --- 視点（積分は GameDataInputCB 側で実施済み。ここでは保持値を読むだけ）
  const rabcl::Attitude view = view_.Get();
  rabbits_interface::SetJointPosition(cmd, YAW_JOINT, view.yaw);
  rabbits_interface::SetJointPosition(cmd, PITCH_JOINT, view.pitch);

  // --- 動作モード
  uint8_t fire = 0;
  uint8_t load = 0;
  if (in.mouse_right_button) {
    fire = 1;
    if (in.mouse_left_button && CanLoad()) {
      load = 1;
    }
  }
  if (in.key_r) {
    load = 2;
  }
  rabbits_interface::SetMode(cmd, MODE_FIRE, fire);
  rabbits_interface::SetMode(cmd, MODE_LOAD, load);
  rabbits_interface::SetMode(cmd, MODE_SPEED, walk ? 0 : 1);
  rabbits_interface::SetMode(cmd, MODE_CHASSIS, in.key_space ? 1 : 0);

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
