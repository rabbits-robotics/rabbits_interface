#include <gtest/gtest.h>

#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include "rabbits_interface/game_client_node.hpp"

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

  std::shared_ptr<GameClient> node_;
};

static rabbits_msgs::msg::GameDataInput MakeInput()
{
  return rabbits_msgs::msg::GameDataInput();
}

TEST_F(GameClientTest, KeyWSetsChassisFwd)
{
  auto input = MakeInput();
  input.key_w = true;
  node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(1.0, cmd.chassis_vel.x);
}

TEST_F(GameClientTest, KeySSetsChassisBwd)
{
  auto input = MakeInput();
  input.key_s = true;
  node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(-1.0, cmd.chassis_vel.x);
}

TEST_F(GameClientTest, KeyASetsChassisLeft)
{
  auto input = MakeInput();
  input.key_a = true;
  node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(1.0, cmd.chassis_vel.y);
}

TEST_F(GameClientTest, KeyDSetsChassisRight)
{
  auto input = MakeInput();
  input.key_d = true;
  node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(-1.0, cmd.chassis_vel.y);
}

TEST_F(GameClientTest, KeyZSetsChassisRotCcw)
{
  auto input = MakeInput();
  input.key_z = true;
  node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(2.0, cmd.chassis_vel.z);
}

TEST_F(GameClientTest, KeyCSetsChassisRotCw)
{
  auto input = MakeInput();
  input.key_c = true;
  node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(-2.0, cmd.chassis_vel.z);
}

TEST_F(GameClientTest, MouseDeltaXSetsYaw)
{
  auto input = MakeInput();
  input.mouse_delta_x = 400;  // yaw_pos = 0 + 400/400 = 1.0
  node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(1.0, cmd.yaw_pos);
}

TEST_F(GameClientTest, YawWrapsNegative)
{
  auto input = MakeInput();
  input.mouse_delta_x = -1000;  // yaw = -10, fmod → negative → add 2π
  node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_GE(cmd.yaw_pos, 0.0);
}

TEST_F(GameClientTest, PitchClampedLow)
{
  auto input = MakeInput();
  input.mouse_delta_y = -1000;  // pitch = -10 → clamped to -M_PI/12
  node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_NEAR(-M_PI / 12, cmd.pitch_pos, 1e-6);
}

TEST_F(GameClientTest, PitchClampedHigh)
{
  auto input = MakeInput();
  input.mouse_delta_y = 1000;  // pitch = 10 → clamped to M_PI/6
  node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_NEAR(M_PI / 6, cmd.pitch_pos, 1e-6);
}

TEST_F(GameClientTest, MouseRightButtonSetsFire)
{
  auto input = MakeInput();
  input.mouse_right_button = true;
  node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_EQ(1, cmd.fire_mode);
}

TEST_F(GameClientTest, MouseLeftButtonSetsLoad)
{
  auto input = MakeInput();
  input.mouse_right_button = true;  // load requires right button held
  input.mouse_left_button = true;
  node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_EQ(1, cmd.load_mode);
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
  node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_EQ(1, cmd.load_mode);
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
  node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_EQ(0, cmd.load_mode);
  // 射撃自体は抑制しない
  EXPECT_EQ(1, cmd.fire_mode);
}

// 熱量データ未受信(max_heat==0)のときはリミットを適用しない
TEST_F(GameClientTest, LoadAllowedWhenNoRobotData)
{
  auto input = MakeInput();
  input.mouse_right_button = true;
  input.mouse_left_button = true;
  node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_EQ(1, cmd.load_mode);
}

TEST_F(GameClientTest, KeyRSetsLoad2)
{
  auto input = MakeInput();
  input.key_r = true;
  node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_EQ(2, cmd.load_mode);
}

TEST_F(GameClientTest, KeyShiftSetsChassisModeOn)
{
  auto input = MakeInput();
  input.key_shift = true;
  node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_EQ(1, cmd.chassis_mode);
}

TEST_F(GameClientTest, SpeedModeAlwaysZero)
{
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_EQ(0, cmd.speed_mode);
}

TEST_F(GameClientTest, UpdatePositionsRuns)
{
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_NO_THROW(node_->UpdatePositions(cmd));
}

TEST_F(GameClientTest, GameDataInputCBSetsMsg)
{
  auto input = MakeInput();
  input.key_w = true;
  node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(1.0, cmd.chassis_vel.x);
}

// マウスは「受信フレームごとに1回だけ」積分される（timer の多重積分が起きない）
TEST_F(GameClientTest, TimerDoesNotReintegrateMouse)
{
  auto input = MakeInput();
  input.mouse_delta_x = 400;  // 1フレームで yaw = 1.0
  node_->GameDataInputCB(std::make_shared<rabbits_msgs::msg::GameDataInput>(input));

  rabbits_msgs::msg::AttractsCommand cmd1;
  node_->UpdateCmdVel(cmd1);
  EXPECT_DOUBLE_EQ(1.0, cmd1.yaw_pos);

  // 同じ入力のまま timer が複数回回っても再積分されない
  rabbits_msgs::msg::AttractsCommand cmd2;
  node_->UpdateCmdVel(cmd2);
  rabbits_msgs::msg::AttractsCommand cmd3;
  node_->UpdateCmdVel(cmd3);
  EXPECT_DOUBLE_EQ(1.0, cmd3.yaw_pos);
}

// フレームを受信するたびに累積する
TEST_F(GameClientTest, RepeatedFramesAccumulateYaw)
{
  auto input = MakeInput();
  input.mouse_delta_x = 400;
  auto msg = std::make_shared<rabbits_msgs::msg::GameDataInput>(input);
  node_->GameDataInputCB(msg);
  node_->GameDataInputCB(msg);  // 2フレーム受信 → yaw = 2.0

  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(2.0, cmd.yaw_pos);
}
