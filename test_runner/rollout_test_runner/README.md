# rollout_test_runner

Diffusion Planner などの End-to-End モデルを closed-loop で評価するための「Rollout simulator」を
scenario_simulator_v2 上に実装したものです。

## 概要

1. **Replay phase**: rosbag（実走行ログ）の ego 状態（`/localization/kinematic_state`,
   `/localization/acceleration`）を再生しながら Autoware の Planning/Control を動かします。
   ego の位置・速度は bag の値で駆動され、Autoware は control command を出力しますが
   ego には反映されません（open-loop）。
2. **切替**: `/rollout/switch`（`std_srvs/srv/Trigger`）を呼ぶと closed-loop へ切り替わります。
3. **Closed-loop phase**: 切替時点の ego 状態（位置・速度・操舵・加速度）は replay 中に毎フレーム
   vehicle model にシード済みのため、以降は Autoware の control command を vehicle model が
   積分して ego 状態を生成します（通常の scenario_simulator_v2 の動作）。
4. **Perception パススルー**: bag に記録された perception 関連トピック（`passthrough_topics`
   パラメータで設定）を、元のタイムスタンプのままシミュレーション時刻に同期して再 publish し続けます
   （切替後も継続）。

## 仕組み

- シミュレーションの ROS 時刻（`/clock`）を bag の時刻に一致させるため、全ノードを
  `use_sim_time:=true` で起動し、`SimulationClock` の初期時刻を bag 開始時刻に設定します。
  これによりパススルーのメッセージは restamp 不要で、concealer が publish する tf と
  bag 内の perception の stamp が整合します。
- Replay 中は `EgoEntity::setControlledBySimulator(true)` + `setMapPose/setTwist/setAcceleration`
  により、simple_sensor_simulator 側の `EgoEntitySimulation::overwrite()` が毎フレーム
  vehicle model に状態を書き込みます。Autoware への `/localization/kinematic_state` 等の供給は
  常に concealer 経由なので、切替時にトピック供給元の変更はありません。
- 切替はフラグを下げるだけで、次フレームから `EgoEntitySimulation::update()` が
  control command の積分を開始します。状態の連続性は構造的に保証されます。

## 使い方

```bash
ros2 launch rollout_test_runner rollout_test.launch.py \
  bag_path:=/path/to/bag \
  map_path:=/path/to/map \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit \
  goal_pose:="[3799.6, 73815.2, 0.0, 0.0, 0.0, 0.47]" \
  launch_rviz:=true

# Autoware が engage したら任意のタイミングで切替
ros2 service call /rollout/switch std_srvs/srv/Trigger
```

### 主要な launch 引数

| 引数 | 既定値 | 説明 |
|---|---|---|
| `bag_path` | （必須） | 再生する rosbag ディレクトリ |
| `map_path` | （必須） | lanelet2 (.osm) と点群 (.pcd) を含む地図ディレクトリ |
| `goal_pose` | `[]` | map 座標系のゴール `[x, y, z, roll, pitch, yaw]`。未指定だと Autoware が engage せず切替できない |
| `replay_start_offset` | `0.0` | bag 先頭からの再生開始オフセット [s] |
| `auto_switch_time` | `-1.0` | この scenario 時刻 [s] で自動切替（負値で無効、バッチ評価用） |
| `rollout_param_file` | `config/rollout.param.yaml` | `passthrough_topics` 等の設定ファイル |
| `launch_autoware` | `true` | Autoware を起動するか |

### パラメータ（rollout.param.yaml）

- `passthrough_topics`: bag からパススルー再生するトピックのリスト。
  `/tf`, `/tf_static`, `/clock`, `/localization/*`, `/vehicle/status/*` はシミュレータが
  供給するため指定しても自動的に除外されます（警告が出ます）。
- `odometry_topic` / `acceleration_topic`: replay に使う ego 状態のトピック名。
- `on_bag_end`: bag の ego 状態が尽きたときの挙動（`hold`: 最終状態を保持 / `switch`: 自動切替）。

## vehicle model に関する注意

Replay 中の状態シードは vehicle model の状態量に書き込まれるため、**速度を状態量に持つモデル**
（`DELAY_STEER_*`, `IDEAL_STEER_ACC*`, `TAIGA_DYN`, `TAIGA_X`）を使ってください。
`IDEAL_STEER_VEL` は速度が入力（状態量でない）のため、replay 中に Autoware へ供給される
`/localization/kinematic_state` の速度が 0 になります。
`vehicle_model_type` は `<vehicle_model>_description/config/simulator_model.param.yaml` で
指定されます（例: `sample_vehicle` は `DELAY_STEER_ACC_GEARED`）。

## 切替条件

`/rollout/switch` は以下の場合に拒否されます:

- すでに closed-loop phase のとき
- Autoware が engage していないとき（control command が信頼できないため）
