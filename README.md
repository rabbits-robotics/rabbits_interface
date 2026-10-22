# rabbits_interface

[![CI](https://github.com/rabbits-robotics/rabbits_interface/actions/workflows/ci.yml/badge.svg)](https://github.com/rabbits-robotics/rabbits_interface/actions/workflows/ci.yml)

人の操作（ゲームパッド、ゲーム側の入力）を機体への指令 `rabbits_msgs/ReferenceCommand` に変換するノード。機体への送信は [rabbits_bridge](https://github.com/rabbits-robotics/rabbits_bridge) の `robot_bridge` が行う。

| ノード | 入力 | 出力 |
|---|---|---|
| `gamepad_node` | `joy`（`sensor_msgs/Joy`、joy_linux） | `reference_command` |
| `game_client_node` | `/game_data_input`、`/game_data_robot` | `reference_command` |

`ReferenceCommand` に入れるもの:

- `chassis_vel`: 車体の並進（x, y [m/s]）と回転（z [rad/s]）
- 関節: `yaw_joint` / `pitch_joint` の目標位置 [rad]（スティックやマウスの相対移動を積分した値）
- モード: `load` / `fire` / `speed` / `chassis`

関節名は URDF、モード名は bridge の YAML（`config/<機体>.yaml`）と同じにする。共通の組み立ては `include/rabbits_interface/reference_builder.hpp`。

パラメータ（launch の引数）: `max_omni_vel` [m/s]、`max_omni_rot_vel` [rad/s]、`max_yaw_rot_vel`、`max_pitch_rot_vel`、`fire_heat`（game_client のみ）。

```bash
ros2 launch rabbits_interface gamepad.launch.py
ros2 launch rabbits_interface game_client.launch.py
```

関節の実際の位置（`joint_states`）は機体のフィードバックから `robot_bridge` が出すので、ここでは出さない。

## テスト

```bash
colcon build --packages-up-to rabbits_interface
colcon test --packages-select rabbits_interface
colcon test-result --verbose
```
