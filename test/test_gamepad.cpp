#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>

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
    MakeNode(-1, -1);
  }

  // lean_left_button / lean_right_button を指定してノードを作り直す
  void MakeNode(int lean_left, int lean_right)
  {
    node_.reset();
    auto options = rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("max_omni_vel", 1.0),
      rclcpp::Parameter("max_omni_rot_vel", 2.0),
      rclcpp::Parameter("max_yaw_rot_vel", 3.0),
      rclcpp::Parameter("max_pitch_rot_vel", 3.0),
      rclcpp::Parameter("walk_scale", 0.4),
      rclcpp::Parameter("body_height_default", 0.5),
      rclcpp::Parameter("lean_roll", 0.15),
      rclcpp::Parameter("lean_left_button", lean_left),
      rclcpp::Parameter("lean_right_button", lean_right),
    });
    node_ = std::make_shared<Gamepad>(options);
  }

  void TearDown() override
  {
    node_.reset();
  }

  // DualShock 4（joy_linux）: axes 8、buttons 13。背面ボタン付きの機種では buttons が増える
  static sensor_msgs::msg::Joy MakeJoy()
  {
    sensor_msgs::msg::Joy joy;
    joy.axes.resize(8, 0.0f);
    joy.buttons.resize(13, 0);
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

TEST_F(GamepadTest, LeftStickSetsChassis)
{
  auto joy = MakeJoy();
  joy.axes[1] = 0.5f;
  joy.axes[0] = -0.5f;
  const auto cmd = Update(joy);
  EXPECT_DOUBLE_EQ(0.5, cmd.chassis_vel.x);
  EXPECT_DOUBLE_EQ(-0.5, cmd.chassis_vel.y);
}

TEST_F(GamepadTest, TriggersSetRotation)
{
  auto joy = MakeJoy();
  joy.axes[2] = 1.0f;   // L2 離し
  joy.axes[5] = -1.0f;  // R2 全押し
  EXPECT_DOUBLE_EQ(2.0, Update(joy).chassis_vel.z);
}

TEST_F(GamepadTest, CrossSetsInfiniteRotation)
{
  auto joy = MakeJoy();
  joy.buttons[0] = 1;
  EXPECT_EQ(1, GetMode(Update(joy), "chassis"));
}

TEST_F(GamepadTest, TriangleWalks)
{
  auto joy = MakeJoy();
  joy.axes[7] = 1.0f;
  joy.buttons[2] = 1;
  const auto cmd = Update(joy);
  EXPECT_DOUBLE_EQ(0.4, cmd.chassis_vel.x);
  EXPECT_EQ(0, GetMode(cmd, "speed"));
}

TEST_F(GamepadTest, SquareCrouches)
{
  auto joy = MakeJoy();
  joy.buttons[3] = 1;
  EXPECT_FLOAT_EQ(0.0f, Update(joy).body_height);
}

TEST_F(GamepadTest, DefaultIsRunAndDefaultHeight)
{
  const auto cmd = Update(MakeJoy());
  EXPECT_EQ(1, GetMode(cmd, "speed"));
  EXPECT_FLOAT_EQ(0.5f, cmd.body_height);
  EXPECT_FLOAT_EQ(0.0f, cmd.body_roll);
  EXPECT_EQ(1, GetMode(cmd, "stabilize"));
}

TEST_F(GamepadTest, LeanDisabledWhenButtonsUnassigned)
{
  auto joy = MakeJoy();
  for (auto & b : joy.buttons) {
    b = 1;
  }
  EXPECT_FLOAT_EQ(0.0f, Update(joy).body_roll);
}

TEST_F(GamepadTest, LeanButtonsFromParameters)
{
  MakeNode(8, 9);
  auto joy = MakeJoy();
  joy.buttons[8] = 1;
  EXPECT_FLOAT_EQ(-0.15f, Update(joy).body_roll);
  joy.buttons[8] = 0;
  joy.buttons[9] = 1;
  EXPECT_FLOAT_EQ(0.15f, Update(joy).body_roll);
}

TEST_F(GamepadTest, LeanButtonOutOfRangeIsIgnored)
{
  MakeNode(20, 21);  // buttons は 13 個しか無い
  auto joy = MakeJoy();
  EXPECT_FLOAT_EQ(0.0f, Update(joy).body_roll);
}

TEST_F(GamepadTest, Axis3SetsYaw)
{
  auto joy = MakeJoy();
  joy.axes[3] = 10.0f;  // yaw = 0 + 10/10 = 1.0
  EXPECT_DOUBLE_EQ(1.0, GetJointPosition(Update(joy), "yaw_joint"));
}

TEST_F(GamepadTest, YawWrapsToPlusMinusPi)
{
  auto joy = MakeJoy();
  joy.axes[3] = -50.0f;  // yaw = -5.0 → +1.28 に wrap
  const double yaw = GetJointPosition(Update(joy), "yaw_joint");
  EXPECT_NEAR(yaw, -5.0 + 2.0 * M_PI, 1e-5);
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

TEST_F(GamepadTest, RebootResetsView)
{
  auto joy = MakeJoy();
  joy.axes[3] = 10.0f;
  EXPECT_DOUBLE_EQ(1.0, GetJointPosition(Update(joy), "yaw_joint"));
  node_->FirmwareRebootCB(std::make_shared<std_msgs::msg::Empty>());
  joy.axes[3] = 0.0f;
  EXPECT_DOUBLE_EQ(0.0, GetJointPosition(Update(joy), "yaw_joint"));
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

TEST_F(GamepadTest, CircleSetsLoad2)
{
  auto joy = MakeJoy();
  joy.buttons[1] = 1;
  EXPECT_EQ(2, GetMode(Update(joy), "load"));
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
  ASSERT_EQ(cmd.mode_name.size(), 5u);
  EXPECT_EQ(cmd.mode_name[0], "load");
  EXPECT_EQ(cmd.mode_name[1], "fire");
  EXPECT_EQ(cmd.mode_name[2], "speed");
  EXPECT_EQ(cmd.mode_name[3], "chassis");
  EXPECT_EQ(cmd.mode_name[4], "stabilize");
  EXPECT_EQ(cmd.mode_value.size(), 5u);
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
