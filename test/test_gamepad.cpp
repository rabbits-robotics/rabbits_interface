#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rabbits_interface/gamepad_node.hpp"
#include "rabbits_interface/reference_builder.hpp"

using rabbits_interface::GetJointPosition;
using rabbits_interface::GetMode;
using rabbits_msgs::msg::ReferenceCommand;

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

  ReferenceCommand Update(const sensor_msgs::msg::Joy & joy)
  {
    node_->JoyCB(std::make_shared<sensor_msgs::msg::Joy>(joy));
    ReferenceCommand cmd;
    node_->UpdateCmdVel(cmd);
    return cmd;
  }

  std::shared_ptr<Gamepad> node_;
};

TEST_F(GamepadTest, DpadUpSetsChassisFwd)
{
  auto joy = MakeJoy();
  joy.axes[7] = 1.0f;
  EXPECT_DOUBLE_EQ(1.0, Update(joy).chassis_vel.x);
}

TEST_F(GamepadTest, DpadDownSetsChassisBwd)
{
  auto joy = MakeJoy();
  joy.axes[7] = -1.0f;
  EXPECT_DOUBLE_EQ(-1.0, Update(joy).chassis_vel.x);
}

TEST_F(GamepadTest, DpadLeftSetsChassisLeft)
{
  auto joy = MakeJoy();
  joy.axes[6] = 1.0f;
  EXPECT_DOUBLE_EQ(1.0, Update(joy).chassis_vel.y);
}

TEST_F(GamepadTest, DpadRightSetsChassisRight)
{
  auto joy = MakeJoy();
  joy.axes[6] = -1.0f;
  EXPECT_DOUBLE_EQ(-1.0, Update(joy).chassis_vel.y);
}

TEST_F(GamepadTest, Button1SetsChassisModeOn)
{
  auto joy = MakeJoy();
  joy.buttons[1] = 1;
  EXPECT_EQ(1, GetMode(Update(joy), "chassis"));
}

TEST_F(GamepadTest, Axis3SetsYaw)
{
  auto joy = MakeJoy();
  joy.axes[3] = 10.0f;  // yaw = 0 + 10/10 = 1.0
  EXPECT_DOUBLE_EQ(1.0, GetJointPosition(Update(joy), "yaw_joint"));
}

TEST_F(GamepadTest, YawWrapsNegative)
{
  auto joy = MakeJoy();
  joy.axes[3] = -1000.0f;  // large negative → wrap
  EXPECT_GE(GetJointPosition(Update(joy), "yaw_joint"), 0.0);
}

TEST_F(GamepadTest, PitchClampedLow)
{
  auto joy = MakeJoy();
  joy.axes[4] = -1000.0f;  // large negative → clamp to -M_PI/12
  EXPECT_NEAR(-M_PI / 12, GetJointPosition(Update(joy), "pitch_joint"), 1e-6);
}

TEST_F(GamepadTest, PitchClampedHigh)
{
  auto joy = MakeJoy();
  joy.axes[4] = 1000.0f;  // large positive → clamp to M_PI/6
  EXPECT_NEAR(M_PI / 6, GetJointPosition(Update(joy), "pitch_joint"), 1e-6);
}

TEST_F(GamepadTest, FireButtonSetsFireMode)
{
  auto joy = MakeJoy();
  joy.buttons[5] = 1;
  const auto cmd = Update(joy);
  EXPECT_EQ(1, GetMode(cmd, "fire"));
  EXPECT_EQ(1, cmd.mode_value[1]);  // mode_name[1] == "fire"
}

TEST_F(GamepadTest, Button4SetsLoad)
{
  auto joy = MakeJoy();
  joy.buttons[4] = 1;
  EXPECT_EQ(1, GetMode(Update(joy), "load"));
}

TEST_F(GamepadTest, Button0SetsLoad2)
{
  auto joy = MakeJoy();
  joy.buttons[0] = 1;
  EXPECT_EQ(2, GetMode(Update(joy), "load"));
}

TEST_F(GamepadTest, SpeedModeAlwaysZero)
{
  EXPECT_EQ(0, GetMode(Update(MakeJoy()), "speed"));
}

TEST_F(GamepadTest, ReferenceCarriesYawPitchAndModes)
{
  const auto cmd = Update(MakeJoy());
  ASSERT_EQ(cmd.joint_name.size(), 2u);
  EXPECT_EQ(cmd.joint_name[0], "yaw_joint");
  EXPECT_EQ(cmd.joint_name[1], "pitch_joint");
  EXPECT_EQ(cmd.joint_position.size(), 2u);
  EXPECT_EQ(cmd.joint_velocity.size(), 2u);
  EXPECT_EQ(cmd.joint_effort.size(), 2u);
  EXPECT_EQ(cmd.joint_kp.size(), 2u);
  EXPECT_EQ(cmd.joint_kd.size(), 2u);
  ASSERT_EQ(cmd.mode_name.size(), 4u);
  EXPECT_EQ(cmd.mode_name[0], "load");
  EXPECT_EQ(cmd.mode_name[1], "fire");
  EXPECT_EQ(cmd.mode_name[2], "speed");
  EXPECT_EQ(cmd.mode_name[3], "chassis");
  EXPECT_EQ(cmd.mode_value.size(), 4u);
}

TEST_F(GamepadTest, TimerCBWithEmptyJoyNoOp)
{
  // joy_msg_ is default (empty axes/buttons) → condition fails → no crash
  EXPECT_NO_THROW(node_->TimerCB());
}

TEST_F(GamepadTest, TimerCBPublishesReferenceCommand)
{
  ReferenceCommand::SharedPtr received;
  auto sub = node_->create_subscription<ReferenceCommand>(
    "reference_command", 10, [&](ReferenceCommand::SharedPtr msg) {received = msg;});
  node_->JoyCB(std::make_shared<sensor_msgs::msg::Joy>(MakeJoy()));
  node_->TimerCB();
  const auto start = std::chrono::steady_clock::now();
  while (received == nullptr &&
    std::chrono::steady_clock::now() - start < std::chrono::milliseconds(200))
  {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_NE(received, nullptr);
  EXPECT_EQ(received->joint_name.size(), 2u);
}

TEST_F(GamepadTest, NoJointStatesPublished)
{
  int count = 0;
  auto sub = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, [&](sensor_msgs::msg::JointState::SharedPtr) {count++;});
  node_->JoyCB(std::make_shared<sensor_msgs::msg::Joy>(MakeJoy()));
  node_->TimerCB();
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(100)) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(count, 0);
}

TEST_F(GamepadTest, JoyCBSetsJoyMsg)
{
  auto joy = MakeJoy();
  joy.axes[7] = 1.0f;
  EXPECT_DOUBLE_EQ(1.0, Update(joy).chassis_vel.x);
}
