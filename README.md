# rabbits_interface

[![CI](https://github.com/rabbits-robotics/rabbits_interface/actions/workflows/ci.yml/badge.svg)](https://github.com/rabbits-robotics/rabbits_interface/actions/workflows/ci.yml)

人の操作（ゲームパッド、ゲーム側の入力）を機体への指令 `rabbits_msgs/ReferenceCommand` に変換するノード。機体への送信は [rabbits_bridge](https://github.com/rabbits-robotics/rabbits_bridge) の `robot_bridge` が行う。

| ノード | 入力 | 出力 |
|---|---|---|
| `gamepad_node` | `joy`（`sensor_msgs/Joy`、joy_linux） | `reference_command` |
| `game_client_node` | `/game_data_input`、`/game_data_robot` | `reference_command` |

`ReferenceCommand` に入れるもの:

- `chassis_vel`: 車体の並進（x, y [m/s]）と回転（z [rad/s]）。視点（カメラ）基準で、ファームが首 Yaw の角度で車体座標系へ回す
- `body_height` / `body_roll` / `body_pitch`: 車高（0〜1）とリーン（カメラ座標系）
- 関節: `yaw_joint` / `pitch_joint` の視点の目標 [rad]（マウスやスティックの相対移動を rabcl の `ViewTarget` で積分。ファームが安定化時は地面基準の向きとして扱う）
- モード: `load` / `fire` / `speed`（0 歩き / 1 走り）/ `chassis`（1 無限回転）/ `stabilize`（常に 1）

関節名は URDF、モード名は bridge の YAML（`config/<機体>.yaml`）と同じにする。共通の組み立ては `include/rabbits_interface/reference_builder.hpp`。

操作の割り当て（機体側の仕様は robot_ws/gamma の README「操作と制御の仕様」）:

| 操作 | キーボード・マウス（`game_client_node`） | ゲームパッド（`gamepad_node`、DualShock 4 の joy_linux 配列） |
|---|---|---|
| 並進 | W / A / S / D | 十字キー（axes 6 / 7）、左スティック（axes 0 / 1） |
| 旋回 | Z / C | L2 / R2（axes 2 / 5） |
| 視点 | マウス | 右スティック（axes 3 / 4） |
| 射出 | 右クリック | R1（button 5） |
| 装填 | 左クリック（右クリック中、熱量に余裕があるとき） | L1（button 4） |
| 逆転 | R | ○（button 1） |
| 無限回転（押している間） | Space | ×（button 0） |
| しゃがみ（押している間） | Shift | □（button 3） |
| 歩き（押している間） | Ctrl | △（button 2） |
| リーン 左 / 右 | Q / E | 背面ボタン（`lean_left_button` / `lean_right_button` に番号を入れる。既定 -1 = 無し） |

bridge の `firmware_reboot`（ファームの再起動検出）を受けたら、視点の目標を 0 に戻す（ファーム側の基準も起動時の向きになるため）。

パラメータ（launch の引数）: `max_omni_vel` [m/s]、`max_omni_rot_vel` [rad/s]、`max_yaw_rot_vel`、`max_pitch_rot_vel`、`walk_scale`（歩きの速度倍率、既定 0.4）、`body_height_default`（既定 0.5）、`lean_roll`（既定 0.15 rad）、`fire_heat`（game_client のみ）、`lean_left_button` / `lean_right_button`（gamepad のみ）。

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
