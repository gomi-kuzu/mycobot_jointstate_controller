# mycobot_jointstate_controller

`sensor_msgs/msg/JointState`を購読し、`pymycobot`経由でmyCobot 280へ関節角コマンドを送るROS2パッケージ

## 機能

- 設定可能なJointStateコマンドトピック（既定値: `/mycobot/joint_commands`）を購読します。
- `get_angles()`で取得した現在の関節角を`/mycobot/joint_states`へpublishします。
- `get_coords()`で取得した手先姿勢を`/mycobot/ee_pose`へpublishします。
- アダプティブグリッパ制御トピックを購読し、開閉やキャリブレーションを実行します。
- JointStateの`name`に基づくmyCobot J1-J6へのマッピングに対応しています。
- 設定可能な安全角度制限でコマンド角をクランプします。
- シリアル通信の過負荷を防ぐため、送信レートを制御できます。

## ビルド

```bash
cd $ROS_HOME
colcon build --packages-select mycobot_jointstate_controller
source install/setup.bash
```

## 起動

```bash
ros2 launch mycobot_jointstate_controller mycobot_jointstate_controller.launch.py \
  port:=/dev/ttyUSB0 \
  command_topic:=/mycobot/joint_commands \
  state_topic:=/mycobot/joint_states \
  ee_pose_topic:=/mycobot/ee_pose \
  gripper_command_topic:=/mycobot/gripper/command \
  gripper_value_topic:=/mycobot/gripper/value \
  gripper_calibration_topic:=/mycobot/gripper/calibrate \
  position_unit:=rad
```

## グリッパ操作例

```bash
# 開く
ros2 topic pub --once /mycobot/gripper/command std_msgs/msg/String "{data: open}"

# 閉じる
ros2 topic pub --once /mycobot/gripper/command std_msgs/msg/String "{data: close}"

# 位置指定（0-100を想定。範囲外はクランプ）
ros2 topic pub --once /mycobot/gripper/value std_msgs/msg/Int32 "{data: 40}"

# キャリブレーション（Emptyメッセージ）
ros2 topic pub --once /mycobot/gripper/calibrate std_msgs/msg/Empty "{}"
```

## 主なパラメータ

- `port` (string): シリアルポート（例: `/dev/ttyUSB0`）
- `baud` (int): ボーレート（既定値: `115200`）
- `command_topic` (string): 購読するコマンドトピック（既定値: `/mycobot/joint_commands`）
- `state_topic` (string): 現在関節角をpublishするトピック（既定値: `/mycobot/joint_states`）
- `ee_pose_topic` (string): 手先姿勢をpublishするトピック（既定値: `/mycobot/ee_pose`）
- `gripper_command_topic` (string): 文字列コマンド購読トピック（既定値: `/mycobot/gripper/command`）
- `gripper_value_topic` (string): 開度値購読トピック（既定値: `/mycobot/gripper/value`）
- `gripper_calibration_topic` (string): キャリブレーション購読トピック（既定値: `/mycobot/gripper/calibrate`）
- `state_publish_rate_hz` (float): `get_angles()`と`get_coords()`の状態publish周期
- `position_unit` (string): `rad`または`deg`（既定値: `rad`）
- `joint_names` (string[6]): JointState上の名前とmyCobot J1..J6の対応
- `joint_angle_min_deg` (float[6]): 各軸の下限角度（度）
- `joint_angle_max_deg` (float[6]): 各軸の上限角度（度）
- `command_speed` (int): `pymycobot`の速度指定値 (既定値: 30%)
- `gripper_speed` (int): `pymycobot`のグリッパ速度指定値 (既定値: 30%)
- `gripper_value_min` (int): グリッパ開度の最小値（既定値: 0）
- `gripper_value_max` (int): グリッパ開度の最大値（既定値: 100）
- `command_rate_hz` (float): シリアル送信周期（既定値: `30.0`）
- `send_only_on_change` (bool): 十分な変化があるときのみ送信
- `change_threshold_deg` (float): 変化量しきい値（度）


## 注意事項

- ROS 2で使用しているPython環境に`pymycobot`と`pyserial`がインストールされていることを確認してください。
- JointStateの名前が異なる場合は`joint_names`を上書きしてください。
- `msg.name`が空の場合、先頭6要素の`position`をJ1..J6として扱います。
- `get_coords()`は`[x, y, z, rx, ry, rz]`（mm/deg）として解釈し、PoseStamped（m + quaternion）へ変換してpublishします。
- `gripper_command_topic`の対応コマンドは`open`/`close`/`calibrate`/`init`/`stop`です。
