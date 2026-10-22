// ReferenceCommand（PC → ファームの汎用指令）を組み立てる小道具。
// gamepad / game_client が共通で使う。関節名は URDF、モード名は bridge の YAML と同じにする。
#ifndef RABBITS_INTERFACE__REFERENCE_BUILDER_HPP_
#define RABBITS_INTERFACE__REFERENCE_BUILDER_HPP_

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include <rabbits_msgs/msg/reference_command.hpp>

namespace rabbits_interface
{

constexpr const char * YAW_JOINT = "yaw_joint";
constexpr const char * PITCH_JOINT = "pitch_joint";
constexpr const char * MODE_LOAD = "load";
constexpr const char * MODE_FIRE = "fire";
constexpr const char * MODE_SPEED = "speed";
constexpr const char * MODE_CHASSIS = "chassis";

// 砲塔 2 関節（位置指令のみ）と 4 モードを 0 で用意する
inline void InitReference(rabbits_msgs::msg::ReferenceCommand & cmd)
{
  cmd.joint_name = {YAW_JOINT, PITCH_JOINT};
  const size_t n = cmd.joint_name.size();
  cmd.joint_position.assign(n, 0.0f);
  cmd.joint_velocity.assign(n, 0.0f);
  cmd.joint_effort.assign(n, 0.0f);
  cmd.joint_kp.assign(n, 0.0f);
  cmd.joint_kd.assign(n, 0.0f);
  cmd.mode_name = {MODE_LOAD, MODE_FIRE, MODE_SPEED, MODE_CHASSIS};
  cmd.mode_value.assign(cmd.mode_name.size(), 0);
}

inline int IndexOf(const std::vector<std::string> & names, const std::string & name)
{
  const auto it = std::find(names.begin(), names.end(), name);
  return it == names.end() ? -1 : static_cast<int>(it - names.begin());
}

inline void SetJointPosition(
  rabbits_msgs::msg::ReferenceCommand & cmd, const std::string & name, double position)
{
  const int i = IndexOf(cmd.joint_name, name);
  if (i >= 0) {
    cmd.joint_position[i] = static_cast<float>(position);
  }
}

inline double GetJointPosition(
  const rabbits_msgs::msg::ReferenceCommand & cmd, const std::string & name)
{
  const int i = IndexOf(cmd.joint_name, name);
  return i >= 0 ? static_cast<double>(cmd.joint_position[i]) : 0.0;
}

inline void SetMode(
  rabbits_msgs::msg::ReferenceCommand & cmd, const std::string & name, uint8_t value)
{
  const int i = IndexOf(cmd.mode_name, name);
  if (i >= 0) {
    cmd.mode_value[i] = value;
  }
}

inline uint8_t GetMode(const rabbits_msgs::msg::ReferenceCommand & cmd, const std::string & name)
{
  const int i = IndexOf(cmd.mode_name, name);
  return i >= 0 ? cmd.mode_value[i] : 0;
}

}  // namespace rabbits_interface

#endif  // RABBITS_INTERFACE__REFERENCE_BUILDER_HPP_
