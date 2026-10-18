#include <gtest/gtest.h>

#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include "rabbits_interface/gamepad_node.hpp"

class GamepadTest : public ::testing::Test
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
    node_ = std::make_shared<Gamepad>(options);
  }

  void TearDown() override
  {
    node_.reset();
  }

  // Build a minimal Joy message with 8 axes and 8 buttons (all zero)
  static sensor_msgs::msg::Joy MakeJoy()
  {
    sensor_msgs::msg::Joy joy;
    joy.axes.resize(8, 0.0f);
    joy.buttons.resize(8, 0);
    return joy;
  }

  std::shared_ptr<Gamepad> node_;
};

TEST_F(GamepadTest, DpadUpSetsChassisFwd)
{
  auto joy = MakeJoy();
  joy.axes[7] = 1.0f;
  node_->JoyCB(std::make_shared<sensor_msgs::msg::Joy>(joy));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(1.0, cmd.chassis_vel.x);
}

TEST_F(GamepadTest, DpadDownSetsChassisBwd)
{
  auto joy = MakeJoy();
  joy.axes[7] = -1.0f;
  node_->JoyCB(std::make_shared<sensor_msgs::msg::Joy>(joy));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(-1.0, cmd.chassis_vel.x);
}

TEST_F(GamepadTest, DpadLeftSetsChassisLeft)
{
  auto joy = MakeJoy();
  joy.axes[6] = 1.0f;
  node_->JoyCB(std::make_shared<sensor_msgs::msg::Joy>(joy));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(1.0, cmd.chassis_vel.y);
}

TEST_F(GamepadTest, DpadRightSetsChassisRight)
{
  auto joy = MakeJoy();
  joy.axes[6] = -1.0f;
  node_->JoyCB(std::make_shared<sensor_msgs::msg::Joy>(joy));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(-1.0, cmd.chassis_vel.y);
}

TEST_F(GamepadTest, Button1SetsChassisModeOn)
{
  auto joy = MakeJoy();
  joy.buttons[1] = 1;
  node_->JoyCB(std::make_shared<sensor_msgs::msg::Joy>(joy));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_EQ(1, cmd.chassis_mode);
}

TEST_F(GamepadTest, Axis3SetsYaw)
{
  auto joy = MakeJoy();
  joy.axes[3] = 10.0f;  // yaw_pos = 0 + 10/10 = 1.0
  node_->JoyCB(std::make_shared<sensor_msgs::msg::Joy>(joy));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(1.0, cmd.yaw_pos);
}

TEST_F(GamepadTest, YawWrapsNegative)
{
  auto joy = MakeJoy();
  joy.axes[3] = -1000.0f;  // large negative → wrap
  node_->JoyCB(std::make_shared<sensor_msgs::msg::Joy>(joy));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_GE(cmd.yaw_pos, 0.0);
}

TEST_F(GamepadTest, PitchClampedLow)
{
  auto joy = MakeJoy();
  joy.axes[4] = -1000.0f;  // large negative → clamp to -M_PI/12
  node_->JoyCB(std::make_shared<sensor_msgs::msg::Joy>(joy));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_NEAR(-M_PI / 12, cmd.pitch_pos, 1e-6);
}

TEST_F(GamepadTest, PitchClampedHigh)
{
  auto joy = MakeJoy();
  joy.axes[4] = 1000.0f;  // large positive → clamp to M_PI/6
  node_->JoyCB(std::make_shared<sensor_msgs::msg::Joy>(joy));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_NEAR(M_PI / 6, cmd.pitch_pos, 1e-6);
}

TEST_F(GamepadTest, Button5SetsFireMode)
{
  auto joy = MakeJoy();
  joy.buttons[5] = 1;
  node_->JoyCB(std::make_shared<sensor_msgs::msg::Joy>(joy));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_EQ(1, cmd.fire_mode);
}

TEST_F(GamepadTest, Button4SetsLoad)
{
  auto joy = MakeJoy();
  joy.buttons[4] = 1;
  node_->JoyCB(std::make_shared<sensor_msgs::msg::Joy>(joy));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_EQ(1, cmd.load_mode);
}

TEST_F(GamepadTest, Button0SetsLoad2)
{
  auto joy = MakeJoy();
  joy.buttons[0] = 1;
  node_->JoyCB(std::make_shared<sensor_msgs::msg::Joy>(joy));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_EQ(2, cmd.load_mode);
}

TEST_F(GamepadTest, SpeedModeAlwaysZero)
{
  auto joy = MakeJoy();
  node_->JoyCB(std::make_shared<sensor_msgs::msg::Joy>(joy));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_EQ(0, cmd.speed_mode);
}

TEST_F(GamepadTest, UpdatePositionsRuns)
{
  auto joy = MakeJoy();
  node_->JoyCB(std::make_shared<sensor_msgs::msg::Joy>(joy));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_NO_THROW(node_->UpdatePositions(cmd));
}

TEST_F(GamepadTest, TimerCBWithEmptyJoyNoOp)
{
  // joy_msg_ is default (empty axes/buttons) → condition fails → no crash
  EXPECT_NO_THROW(node_->TimerCB());
}

TEST_F(GamepadTest, TimerCBWithJoyExecutes)
{
  auto joy = MakeJoy();
  node_->JoyCB(std::make_shared<sensor_msgs::msg::Joy>(joy));
  EXPECT_NO_THROW(node_->TimerCB());
}

TEST_F(GamepadTest, JoyCBSetsJoyMsg)
{
  auto joy = MakeJoy();
  joy.axes[7] = 1.0f;
  node_->JoyCB(std::make_shared<sensor_msgs::msg::Joy>(joy));
  rabbits_msgs::msg::AttractsCommand cmd;
  node_->UpdateCmdVel(cmd);
  EXPECT_DOUBLE_EQ(1.0, cmd.chassis_vel.x);
}
