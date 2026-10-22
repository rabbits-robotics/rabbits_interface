#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rabbits_interface/game_client_node.hpp"
#include "rabbits_interface/reference_builder.hpp"

using rabbits_interface::GetJointPosition;
using rabbits_interface::GetMode;
using rabbits_msgs::msg::ReferenceCommand;

class GameClientTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    auto options = rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("max_omni_vel", 1.0),
      rclcpp::Parameter("max_omni_rot_vel", 2.0),
      rclcpp::Parameter("max_yaw_rot_vel", 3.0),
      rclcpp::Parameter("max_pitch_rot_vel", 3.0),
    });
    node_ = std::make_shared<GameClient>(options);
  }

  void TearDown() override
  {
    node_.reset();
  }

  static rabbits_msgs::msg::GameDataInput MakeInput()
  {
    return rabbits_msgs::msg::GameDataInput();
  }

  ReferenceCommand Update(const rabbits_msgs::msg::GameDataInput & input)
  {
    node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));
    ReferenceCommand cmd;
    node_->UpdateCmdVel(cmd);
    return cmd;
  }

  std::shared_ptr<GameClient> node_;
};

TEST_F(GameClientTest, KeyWSetsChassisFwd)
{
  auto input = MakeInput();
  input.key_w = true;
  EXPECT_DOUBLE_EQ(1.0, Update(input).chassis_vel.x);
}

TEST_F(GameClientTest, KeySSetsChassisBwd)
{
  auto input = MakeInput();
  input.key_s = true;
  EXPECT_DOUBLE_EQ(-1.0, Update(input).chassis_vel.x);
}

TEST_F(GameClientTest, KeyASetsChassisLeft)
{
  auto input = MakeInput();
  input.key_a = true;
  EXPECT_DOUBLE_EQ(1.0, Update(input).chassis_vel.y);
}

TEST_F(GameClientTest, KeyDSetsChassisRight)
{
  auto input = MakeInput();
  input.key_d = true;
  EXPECT_DOUBLE_EQ(-1.0, Update(input).chassis_vel.y);
}

TEST_F(GameClientTest, KeyZSetsChassisRotCcw)
{
  auto input = MakeInput();
  input.key_z = true;
  EXPECT_DOUBLE_EQ(2.0, Update(input).chassis_vel.z);
}

TEST_F(GameClientTest, KeyCSetsChassisRotCw)
{
  auto input = MakeInput();
  input.key_c = true;
  EXPECT_DOUBLE_EQ(-2.0, Update(input).chassis_vel.z);
}

TEST_F(GameClientTest, MouseDeltaXSetsYaw)
{
  auto input = MakeInput();
  input.mouse_delta_x = 400;  // yaw = 0 + 400/400 = 1.0
  EXPECT_DOUBLE_EQ(1.0, GetJointPosition(Update(input), "yaw_joint"));
}

TEST_F(GameClientTest, YawWrapsNegative)
{
  auto input = MakeInput();
  input.mouse_delta_x = -1000;  // yaw = -10, fmod → negative → add 2π
  EXPECT_GE(GetJointPosition(Update(input), "yaw_joint"), 0.0);
}

TEST_F(GameClientTest, PitchClampedLow)
{
  auto input = MakeInput();
  input.mouse_delta_y = -1000;  // pitch = -10 → clamped to -M_PI/12
  EXPECT_NEAR(-M_PI / 12, GetJointPosition(Update(input), "pitch_joint"), 1e-6);
}

TEST_F(GameClientTest, PitchClampedHigh)
{
  auto input = MakeInput();
  input.mouse_delta_y = 1000;  // pitch = 10 → clamped to M_PI/6
  EXPECT_NEAR(M_PI / 6, GetJointPosition(Update(input), "pitch_joint"), 1e-6);
}

TEST_F(GameClientTest, MouseRightButtonSetsFire)
{
  auto input = MakeInput();
  input.mouse_right_button = true;
  EXPECT_EQ(1, GetMode(Update(input), "fire"));
}

TEST_F(GameClientTest, MouseLeftButtonSetsLoad)
{
  auto input = MakeInput();
  input.mouse_right_button = true;  // load requires right button held
  input.mouse_left_button = true;
  EXPECT_EQ(1, GetMode(Update(input), "load"));
}

TEST_F(GameClientTest, LoadAllowedWhenHeatLow)
{
  rabbits_msgs::msg::GameDataRobot robot;
  robot.max_heat = 60;
  robot.current_heat = 0;
  node_->GameDataRobotCB(std::make_shared<rabbits_msgs::msg::GameDataRobot>(robot));

  auto input = MakeInput();
  input.mouse_right_button = true;
  input.mouse_left_button = true;
  EXPECT_EQ(1, GetMode(Update(input), "load"));
}

// 次の射撃で熱量が最大に達する場合は装填を抑制する（fire_heat=10, max=60 → 50で抑制）
TEST_F(GameClientTest, LoadBlockedWhenHeatNearMax)
{
  rabbits_msgs::msg::GameDataRobot robot;
  robot.max_heat = 60;
  robot.current_heat = 50;
  node_->GameDataRobotCB(std::make_shared<rabbits_msgs::msg::GameDataRobot>(robot));

  auto input = MakeInput();
  input.mouse_right_button = true;
  input.mouse_left_button = true;
  const auto cmd = Update(input);
  EXPECT_EQ(0, GetMode(cmd, "load"));
  // 射撃自体は抑制しない
  EXPECT_EQ(1, GetMode(cmd, "fire"));
}

// 熱量データ未受信(max_heat==0)のときはリミットを適用しない
TEST_F(GameClientTest, LoadAllowedWhenNoRobotData)
{
  auto input = MakeInput();
  input.mouse_right_button = true;
  input.mouse_left_button = true;
  EXPECT_EQ(1, GetMode(Update(input), "load"));
}

TEST_F(GameClientTest, KeyRSetsLoad2)
{
  auto input = MakeInput();
  input.key_r = true;
  EXPECT_EQ(2, GetMode(Update(input), "load"));
}

TEST_F(GameClientTest, KeyShiftSetsChassisModeOn)
{
  auto input = MakeInput();
  input.key_shift = true;
  EXPECT_EQ(1, GetMode(Update(input), "chassis"));
}

TEST_F(GameClientTest, SpeedModeAlwaysZero)
{
  ReferenceCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_EQ(0, GetMode(cmd, "speed"));
}

TEST_F(GameClientTest, ReferenceCarriesYawPitchAndModes)
{
  ReferenceCommand cmd;
  node_->UpdateCmdVel(cmd);
  ASSERT_EQ(cmd.joint_name.size(), 2u);
  EXPECT_EQ(cmd.joint_name[0], "yaw_joint");
  EXPECT_EQ(cmd.joint_name[1], "pitch_joint");
  EXPECT_EQ(cmd.joint_kd.size(), 2u);
  ASSERT_EQ(cmd.mode_name.size(), 4u);
  EXPECT_EQ(cmd.mode_value.size(), 4u);
}

TEST_F(GameClientTest, NoJointStatesPublished)
{
  int count = 0;
  auto sub = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, [&](sensor_msgs::msg::JointState::SharedPtr) {count++;});
  node_->TimerCB();
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(100)) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(count, 0);
}

TEST_F(GameClientTest, GameDataInputCBSetsMsg)
{
  auto input = MakeInput();
  input.key_w = true;
  EXPECT_DOUBLE_EQ(1.0, Update(input).chassis_vel.x);
}

// マウスは「受信フレームごとに1回だけ」積分される（timer の多重積分が起きない）
TEST_F(GameClientTest, TimerDoesNotReintegrateMouse)
{
  auto input = MakeInput();
  input.mouse_delta_x = 400;  // 1フレームで yaw = 1.0
  EXPECT_DOUBLE_EQ(1.0, GetJointPosition(Update(input), "yaw_joint"));

  // 同じ入力のまま timer が複数回回っても再積分されない
  ReferenceCommand cmd2;
  node_->UpdateCmdVel(cmd2);
  ReferenceCommand cmd3;
  node_->UpdateCmdVel(cmd3);
  EXPECT_DOUBLE_EQ(1.0, GetJointPosition(cmd3, "yaw_joint"));
}

// フレームを受信するたびに累積する
TEST_F(GameClientTest, RepeatedFramesAccumulateYaw)
{
  auto input = MakeInput();
  input.mouse_delta_x = 400;
  auto msg = std::make_shared<rabbits_msgs::msg::GameDataInput>(input);
  node_->GameDataInputCB(msg);
  node_->GameDataInputCB(msg);  // 2フレーム受信 → yaw = 2.0

  ReferenceCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(2.0, GetJointPosition(cmd, "yaw_joint"));
}
