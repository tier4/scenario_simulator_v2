# Perception Reproducer Sensor

実車ログ (rosbag) に記録された perception 結果（検出物体・追跡物体・信号認識・占有格子）を
シミュレーション中の Autoware に再生する仕組みです。
`simple_sensor_simulator` 内の擬似センサとして実装されています
（`simulation/simple_sensor_simulator/.../sensor_simulation/perception_reproducer_sensor/`）。

## 背景と目的

NPC を持たない自動生成シナリオ（実車ログから始点・経路だけを抽出したもの）では、
実環境に存在した先行車や信号が再現されないため、実機の「先行車に追従して減速した」
「赤信号で停止した」といった挙動を sim 側で比較できません。

Perception Reproducer Sensor は、実車ログの perception 出力をそのまま
シミュレーション中の Autoware へ publish することで、NPC をモデリングせずに
実環境の周辺コンテキストを再現します。主用途は実機ログと sim の挙動比較
（real_log_sim_comparison ワークフロー）です。

## アーキテクチャ

```
       rosbag (.mcap / bag ディレクトリ)
            │  構築時に全メッセージを一括ロード（時刻キー付き配列）
            ▼
┌─ PerceptionReproducerSensor ──────────────────────────────────┐
│  BagStream<DetectedObjects>   → /perception/.../detection/objects │
│  BagStream<TrackedObjects>    → /perception/.../tracking/objects  │
│  BagStream<Trajectory>        → /simulation/replay/trajectory     │
│  BagStream<OccupancyGrid>     → /perception/occupancy_grid_map/map│
│  TrafficLightBagStream        → /perception/.../traffic_signals   │
│  TFStreamFromOdometry         → TF map→replay_base_link + marker  │
└──────────────────────▲────────────────────────────────────────┘
                       │ update(scenario_time, ros_time, ego_pose, ego_speed)
        SensorSimulation::updateSensorFrame（毎フレーム、ZeroMQ UpdateFrameRequest 起点）
```

### 構成要素

- **`BagStream<T>`** (`bag_stream.hpp/cpp`): 1 トピック分の時刻キー付き再生ストリーム。
  bag ロード時に `(scenario_time, message)` の配列を構築し、
  `publishUpTo()`（時系列再生）と `publishNearest()`（最近傍スナップショット再生）を提供します。
  publish 時に `header.stamp` を現在の ROS 時刻に書き換え、
  `frame_id == "base_link"` のメッセージは `replay_base_link` に付け替えます。
- **`TFStreamFromOdometry`**: bag の `/localization/kinematic_state` から
  `map → replay_base_link` の TF を broadcast します（位置は線形補間、姿勢は slerp）。
  base_link 相対で記録された物体が、記録時の実車位置を基準に配置されるための frame です。
  位置同期モード用に最近傍 odometry サンプル探索 (`findNearestIndex`) も提供します。
- **`TrafficLightBagStream`** (`traffic_light_bag_stream.hpp/cpp`):
  `TrafficLightGroupArray` 用ストリーム。`stamp` を現在 ROS 時刻に書き換える際、
  各 prediction の `predicted_stamp` を元の `stamp` からの相対時間を保ったまま補正します。
  メッセージ型が存在する環境でのみ有効です（`__has_include` による条件コンパイル）。
- **`PerceptionReproducerSensor`** 本体: 上記ストリームを束ね、再生モードの判定・
  信号フィルタ・再生車両の可視化 marker (`/simulation/replay/vehicle_marker`) を担います。

bag の読み込みは `rosbag2_cpp::Reader` によります。bare の `.mcap` ファイルを直接指定した
場合は metadata.yaml が無いため `storage_id = "mcap"` を明示し、ディレクトリ bag
（mcap / sqlite3）は metadata.yaml から自動判別されます。

### 駆動経路

`simple_sensor_simulator` ノードは起動時に `replay_bag_path` パラメータが非空であれば
センサを構築し、同時に `setSuppressDetectionSensor(true)` を呼びます。
これにより**通常の DetectionSensor の update が抑止され**、シナリオ上の NPC 由来の
検出結果と bag 由来の検出結果が混在しません。

毎フレーム `SensorSimulation::updateSensorFrame()` が EGO エンティティの pose と
速度（twist の linear x, y の合成）を取り出して `update()` に渡します。
scenario_time が NaN または負（シナリオ開始前）の間は何もしません。
また `InitializeRequest` 受信時に `reset()` され、再生は先頭からやり直しになります。

## 再生対象トピック

| bag 内トピック | 再生先 | 備考 |
|---|---|---|
| `/perception/object_recognition/detection/objects` | 同一トピック | DetectedObjects |
| `/perception/object_recognition/tracking/objects` | 同一トピック | TrackedObjects。**実車ログに detection が記録されていないケースのため直接再生**します。DetectionSensor 抑止により Autoware の tracker が無入力でも、DiffusionPlanner や map_based_prediction 等の tracking 購読者に実環境の物体が届きます |
| `/planning/trajectory` | `/simulation/replay/trajectory` | 実機 planner 出力の参照用（Autoware には入力しない）。時刻同期モードのみ |
| `/localization/kinematic_state` | TF `map → replay_base_link` | 補間して broadcast。可視化 marker も publish |
| `/perception/occupancy_grid_map/map` | 同一トピック | transient_local QoS、常に最近傍 1 件を publish |
| `/perception/traffic_light_recognition/traffic_signals` | 同一トピック | ego-govern フィルタ適用（後述） |

## 時刻写像の規約

bag 内の各メッセージは以下の scenario 時刻に割り当てられます:

```
scenario_time = (bag 受信時刻 − bag metadata の starting_time) − replay_start_time
```

負になったメッセージ（再生開始点より前）はロード時に捨てられます。
この規約は ego 状態をリプレイする **EgoBagReplayer**（後述）と同一であり、
launch 引数 `replay_start_time` の 1 値だけで perception 再生と ego 再生が同期します。

## 再生モード

```
update()
 ├─ replay_use_position_based = false ──▶ 時刻同期: publishUpTo(scenario_time)
 └─ replay_use_position_based = true
     ├─ 初回 ────────────────▶ 大域最近傍探索で playhead 初期化
     ├─ ego_speed > 0.5 m/s ──▶ 窓 [playhead−50, playhead+600) 内の最近傍
     │                          （後退禁止: max(前回, 探索結果)）
     └─ ego_speed ≤ 0.5 m/s ──▶ dwell anchor から記録実ペースで時間前進
```

### 時刻同期再生（既定）

`publishUpTo()` により「記録時の scenario 時刻 ≤ 現在の scenario 時刻」のメッセージを
順番に publish します。実機のタイムラインに忠実な再生です。
detection / tracking / trajectory の全ストリームを publish し終えたら何もしなくなります。

### 位置同期再生（`replay_use_position_based:=true`）

sim ego の現在位置に最も近い記録 ego 位置（odometry サンプル）を求め、
その記録時刻のスナップショット（各トピックの最近傍 1 件）を publish します。
sim ego の走行ペースが実機とずれても、「その場所で実機が見ていた風景」が再生されます。
以下の 2 つの機構を含みます。

- **単調 playhead + 近傍窓探索**: playhead（odometry サンプル index）は決して後退せず、
  最近傍探索は前回 playhead の後方 50 / 前方 600 サンプルの窓に限定されます
  （odometry 約 50 Hz 前提で後方約 1 秒 / 前方約 12 秒）。周回コースやロータリーのような
  自己近接コースで、大域最近傍が遠い周回弧にマッチして再生がテレポートする・
  逆行することを防ぎます。
- **停止中 time-advance**: sim ego の速度が 0.5 m/s 以下になると、その時点を
  anchor として playhead を**記録の実ペースで時間前進**させます。純粋な位置同期では
  ego 停止中に playhead が凍結し、記録上の先行車が永遠に発進せず ego も解放されない
  デッドロックに陥るためです。実機が経験した「停止 → 先行車発進」の系列を
  そのまま再生します。ego が再び動き出すと anchor は破棄され位置同期に戻ります。
  なお ego 速度が取得できないフレームは「走行中」として扱われます
  （無断で時間前進しないための防御）。

### 使い分けの指針

- **時刻同期**: 実機タイムラインへの忠実さを重視する場合。sim ego が実機と
  同等ペースで走れる前提。
- **位置同期**: sim ego のペースが実機とずれる場合（モデル差・planner 差の比較など）。
  ego の現在位置に周辺コンテキストが追従します。

## ego-govern 信号フィルタ

bag の信号認識には交差方向の信号も含まれます。これをそのまま再生すると、
behavior planner が**交差方向の赤信号を自レーンの停止信号と誤解釈して偽停止**します
（旧 Python サイドカー実装で実際に観測された問題）。

そこで初回 `update()` 時に以下のフィルタを適用します:

1. 記録 ego 軌跡（odometry）を最大 500 点程度に subsample し、各点を
   マッチ距離 5 m で lanelet にマッチして「実走 lanelet 集合」を得る
2. 実走 lanelet 上の信号 regulatory element の group id 集合（governing set）を求める
3. bag の全 `TrafficLightGroupArray` から governing set 外の group を除去する

エッジケースの扱い:

- **governing set が空**（コース上に信号が無い）の場合、再生される信号は
  すべて交差方向のものなので、全除去が正しい挙動です。
- **レーンマッチが全滅**（実走 lanelet 集合が空）の場合のみ、信頼できない集合での
  フィルタは全信号を消してしまうため、防御的にフィルタをスキップして警告を出します。

lanelet map は `InitializeRequest` で activate されるため、センサ構築時ではなく
初回 `update()` での遅延適用になっています。

## パラメータ

`simple_sensor_simulator` ノードのパラメータ:

| パラメータ | 既定値 | 説明 |
|---|---|---|
| `replay_bag_path` | `""` | 再生する bag のパス（ディレクトリまたは bare .mcap）。非空で再生有効化 + DetectionSensor 抑止 |
| `replay_start_time` | `0.0` | bag 先頭からの再生開始オフセット [s] |
| `replay_use_position_based` | `false` | 位置同期再生を使うか |

`scenario_test_runner.launch.py` では `replay_bag_path` / `replay_start_time` /
`replay_ego_duration` が launch 引数として宣言されており、`replay_bag_path` が非空のとき
`make_parameters()` 経由で **simple_sensor_simulator と openscenario_interpreter の
両ノードに渡ります**（perception 再生は前者、ego 再生は後者が使用）。
`replay_use_position_based` は `simple_sensor_simulator.` プレフィックス付き launch 引数
として渡します（プレフィックス以降がそのままノードパラメータになります）。

## EgoBagReplayer との連携

`traffic_simulator/replay/ego_bag_replayer.hpp` の `EgoBagReplayer` は、シナリオ開始から
`replay_ego_duration` 秒間、bag の ego 状態（pose / twist / accel）を補間して ego に
注入（overwrite）し、その後 closed-loop（通常の vehicle model 駆動）に切り替えます。
時刻写像は本センサと同一規約なので、`replay_start_time` を合わせるだけで
ego 再生フェーズ中の perception 再生が実機と同じ対応関係になります。
詳細は `ego_bag_replayer.hpp` のコードコメントを参照してください。

## 使用例

```bash
ros2 launch scenario_test_runner scenario_test_runner.launch.py \
  ... \
  replay_bag_path:=/path/to/bag_dir \
  replay_start_time:=12.0 \
  replay_ego_duration:=8.0 \
  simple_sensor_simulator.replay_use_position_based:=true
```

最初の 8 秒は ego が bag の状態で駆動され（EgoBagReplayer）、以降 closed-loop。
perception は全期間にわたり bag から再生されます。

## 制約・注意点

- **実車ログ側に必要なトピック**: 少なくとも `/localization/kinematic_state` が必要です
  （TF・位置同期・信号フィルタの基準）。detection / tracking / occupancy grid / 信号は
  記録されているものだけが再生されます。
- **信号 topic の所有**: 再生有効時は本センサが
  `/perception/traffic_light_recognition/traffic_signals` に publish するため、
  シナリオ側で信号状態を設定しない運用としてください（同一トピックへの publish が
  競合します）。
- **位置同期モードの限界**: playhead が単調のため、後退走行や、同一地点を再訪する
  コース（記録の別周回への再マッチが必要なケース）には対応できません。
  近傍窓のサンプル幅は odometry 約 50 Hz を前提とした定数です。
- **物体配置の基準**: base_link 相対で記録された物体は `replay_base_link`
  （記録時の実車位置）基準で配置されます。時刻同期モードで sim ego が実機軌跡から
  大きくずれると、ego から見た物体の相対位置は実機のものと一致しません。
- `/simulation/replay/trajectory` は時刻同期モードでのみ publish されます。
- 信号再生（`TrafficLightGroupArray`）はメッセージ型が利用可能な
  architecture でのみコンパイルされます。
