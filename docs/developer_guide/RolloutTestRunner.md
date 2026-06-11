# Rollout Test Runner

Diffusion Planner などの End-to-End モデルを closed-loop で評価するための
「Rollout simulator」を scenario_simulator_v2 (SSV2) 上に実装したものです。
パッケージは `test_runner/rollout_test_runner` にあります。

## 背景と目的

実車ログ（rosbag）には実環境での走行状態と認識結果が記録されていますが、
そのまま再生するだけでは planner / controller の出力が車両に反映されない
open-loop 評価しかできません。一方、最初から全てをシミュレーションすると
実環境とのギャップ（認識結果・走行履歴）が大きくなります。

Rollout simulator は両者を組み合わせます:

1. **Replay phase**: rosbag の ego 走行ログで ego を駆動しながら、
   Autoware の Planning/Control を実走行と同じ入力で動かす（open-loop）。
   Autoware は control command を出力するが、ego には反映されない。
2. **切替**: 任意の時点で ROS サービスにより closed-loop に切替。
3. **Closed-loop phase**: 以降は Autoware の control command を vehicle model が
   積分して ego 状態を生成する（通常の SSV2 動作）。

これにより、実走行ログのある時点から「もしこのままモデルに運転させたらどうなるか」
を、実環境と同一の文脈（速度・位置・周辺認識）から評価できます。
以下のような実機とシミュレーションの乖離の再現・検証を想定しています:

- 実機では時速 40 km まで加速するシーンでも、シミュレーションでは 25 km 程度しか出ない
- planning simulator では停止できるのに、実機では急ブレーキになったりジリジリ進んだりする
- 制御誤差によって停止位置より手前で止まってしまう

## アーキテクチャ

```
┌─────────────────────┐  ZeroMQ   ┌──────────────────────────┐
│ rollout_test_runner │──────────▶│ simple_sensor_simulator  │
│  (traffic_simulator │  Update   │  EgoEntitySimulation     │
│   API を直接使用)   │  Entity   │   ├ overwrite() ←replay  │
│  ├ EgoReplaySource  │  Status   │   └ update()    ←切替後  │
│  ├ BagTopicRepublisher           │  concealer::AutowareUniverse
│  └ /rollout/switch  │           │   └ kinematic_state 等を │
└─────────┬───────────┘           │     Autoware へ publish  │
          │ /clock (bag時刻)      └────────────┬─────────────┘
          │ パススルー (perception等)           │ control_cmd
          ▼                                    ▼
┌──────────────────────────── Autoware ────────────────────────┐
│  Planning / Control（concealer 経由で launch・engage 制御）  │
└──────────────────────────────────────────────────────────────┘
```

重要なのは、**Autoware への tf / kinematic_state / vehicle status の供給は
replay 中も切替後も常に concealer が一元的に担う**ことです。
オリジナルの Rollout simulator（logging_simulator + rosbag 再生 + 途中で
simple_planning_simulator を起動）で必要だった「topic 供給元の切替」や
「一時停止」が存在せず、切替は瞬時かつ連続的に行われます。

### Replay phase の状態注入

毎フレーム、`api_.updateFrame()` の直前に以下を行います:

```cpp
ego.setControlledBySimulator(true);
ego.setMapPose(sample.pose);       // bag の odometry を補間した値
ego.setTwist(sample.twist);
ego.setAcceleration(sample.accel);
```

`API::updateEntitiesStatusInSim()` は `isControlledBySimulator()` が真のとき
`UpdateEntityStatusRequest.overwrite_ego_status = true` を simple_sensor_simulator
に送り、`EgoEntitySimulation::overwrite()` が vehicle model の状態
（位置・ヨー・速度・操舵角・加速度）を直接書き換えます。書き換えられた状態は
`setAutowareStatus()` 経由で `/localization/kinematic_state` 等として Autoware に
届くため、**Autoware の速度フィードバックは replay 中も bag の値で成立します**。

### 切替（closed-loop 化）

`/rollout/switch`（`std_srvs/srv/Trigger`）のハンドラは
`ego.setControlledBySimulator(false)` でフラグを下げるだけです。
次フレームから `EgoEntitySimulation::update()` が control command の積分を
開始しますが、replay の全フレームで vehicle model に状態がシード済みのため、
**位置・速度・操舵・加速度の連続性は構造的に保証されます**。
vehicle model への明示的な再初期化は不要です。

切替は以下の場合に拒否されます（レスポンスの `message` に理由が入ります）:

- すでに closed-loop phase のとき
- Autoware が engage していないとき（control command が信頼できないため）

`auto_switch_time` パラメータで scenario 時刻ベースの自動切替も可能です
（バッチ評価向け）。

### 時刻同期

シミュレーションの ROS 時刻（`/clock`）を bag の時刻に一致させます。

- 全ノードを `use_sim_time:=true` で起動（launch が強制します）
- `SimulationClock` の初期シミュレーション ROS 時刻を
  「bag の最初の odometry の時刻 + `replay_start_offset`」に設定
- フレーム n の ROS 時刻 = `bag_anchor + n × realtime_factor / frame_rate`
  で完全に決定論的（`ros2 bag play` のような実時間依存なし）

この方式により:

- パススルーするメッセージは**元のタイムスタンプのまま** publish できる
  （restamp 不要、Autoware 側の stale 判定や遅延補償と整合）
- concealer が publish する tf（map → base_link）の時刻と bag 内の
  perception メッセージの `header.stamp` が整合する

### Perception パススルー

`BagTopicRepublisher` は bag 内の指定トピックを `rclcpp::GenericPublisher` で
型を問わず再 publish します。読み出しはストリーミング（先読み 1 件バッファ）で、
毎フレーム「記録時刻 ≤ 現在のシミュレーション時刻」のメッセージを全て publish
します。切替後も継続します（要件: Diffusion Planner 等への周辺物体入力の維持）。

以下のトピックはシミュレータ自身が供給するため、指定しても自動的に除外され
警告が出ます: `/tf`, `/tf_static`, `/clock`, `/localization/*`, `/vehicle/status/*`

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

### launch 引数

| 引数 | 既定値 | 説明 |
|---|---|---|
| `bag_path` | （必須） | 再生する rosbag ディレクトリ |
| `map_path` | （必須） | lanelet2 (.osm) と点群 (.pcd) を含む地図ディレクトリ |
| `goal_pose` | `[]` | map 座標系のゴール `[x, y, z, roll, pitch, yaw]`。未指定だと Autoware が engage せず切替不可（警告のみで起動は可能） |
| `replay_start_offset` | `0.0` | bag 先頭からの再生開始オフセット [s] |
| `auto_switch_time` | `-1.0` | この scenario 時刻 [s] で自動切替（負値で無効） |
| `rollout_param_file` | `config/rollout.param.yaml` | パススルー設定等のファイル |
| `vehicle_model` | `""` | 車両モデル（`<vehicle_model>_description` の param を読み込む） |
| `launch_autoware` | `true` | Autoware を起動するか（デバッグ用） |
| `global_frame_rate` | `20.0` | シミュレーションフレームレート [Hz] |
| `global_timeout` | `0.0` | scenario 時刻のタイムアウト [s]（0 で無効） |

### ノードパラメータ（rollout.param.yaml）

| パラメータ | 既定値 | 説明 |
|---|---|---|
| `passthrough_topics` | perception 系 3 トピック | bag からパススルー再生するトピック |
| `transient_local_topics` | （なし） | transient_local QoS で publish するトピック。**Humble の YAML パーサは空配列を表現できない**ため、不要なら記述しないこと |
| `odometry_topic` | `/localization/kinematic_state` | replay に使う ego odometry |
| `acceleration_topic` | `/localization/acceleration` | replay に使う ego 加速度 |
| `on_bag_end` | `hold` | bag の ego 状態が尽きたときの挙動（`hold` / `switch`） |

## vehicle model の選択

Replay 中の状態シードは vehicle model の**状態量**に書き込まれるため、
速度を状態量に持つモデルを使用してください:

| モデル | replay でシードされる量 | 適否 |
|---|---|---|
| `DELAY_STEER_ACC` 系 | 位置・ヨー・速度・操舵・加速度 | ◎ |
| `TAIGA_DYN` / `TAIGA_X` | 上記 + 横速度・ヨーレート | ◎ |
| `IDEAL_STEER_ACC` 系 | 位置・ヨー・速度 | ○ |
| `IDEAL_STEER_VEL` | 位置・ヨーのみ（速度は入力） | ×（速度供給が 0 になる） |

## 実装に伴う既存コードの変更

すべて後方互換です（既存の呼び出しはデフォルト引数で従来動作）。

1. **`SimulationClock`**: コンストラクタに第 4 引数
   `initial_simulation_ros_time`（既定 0.0）を追加。また内部時刻の
   クロック型を `RCL_ROS_TIME` に明示（従来は暗黙に `RCL_SYSTEM_TIME` 型で
   構築されており、`RCL_ROS_TIME` 型の時刻と比較すると例外になる潜在バグ）。
2. **`EgoEntity::onUpdate()`**: `is_controlled_by_simulator_` 分岐の条件に
   `polyline_trajectory_` の null チェックを追加。従来この組み合わせになる
   経路は存在しなかったため動作は不変（潜在的な null デリファレンスの修正）。
3. **`traffic_simulator::API`**: `getCurrentRosTime()` アクセサを追加（純追加）。

## 検証

合成 bag（直線 3 m/s × 20 s）+ kashiwanoha 地図によるスモークテストで以下を確認済み:

- `/clock` が bag 時刻起点で進行する
- ego が bag 軌道を正確に追従する（位置・速度とも bag の値に一致）
- `DELAY_STEER_ACC_GEARED` で `/localization/kinematic_state` の速度が bag の値になる
- パススルートピックがシミュレーション時刻に同期して配信される
- bag 末尾で最終状態をホールドする（`on_bag_end: hold`）
- Autoware 未 engage 時に `/rollout/switch` が拒否される
- 禁止トピック（`/localization/*` 等）の指定が自動除外される
- traffic_simulator に依存する全パッケージのビルドが通る（回帰）

実 bag + Autoware 込みの closed-loop 切替（切替前後の速度連続性）は
実行環境が必要なため、利用時に以下で確認してください:

```bash
ros2 bag record /localization/kinematic_state /control/command/control_cmd
ros2 service call /rollout/switch std_srvs/srv/Trigger
# 切替時刻前後の速度プロットに段差がないこと
```

## 制限事項

- パススルーは記録時の認識結果をそのまま再生するため、切替後に ego が
  元の軌道から大きく離れると周辺認識と実際の ego 位置に矛盾が生じます。
  将来的には bag の認識結果を NPC エンティティとして再現し detection sensor
  経由で供給する方式（ego の位置に追従した認識のシミュレーション）が考えられます。
- ゴールはパラメータ指定のみです（bag の route トピックからの自動抽出は未実装）。
- bag の gear / turn indicator 等は再現しません。
