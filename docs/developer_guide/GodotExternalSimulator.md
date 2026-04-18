# Godot External Simulator 連携ガイド

## 概要

`scenario_simulator_v2` は `VehicleModelType::EXTERNAL` モードを通じて、外部車両シミュレータである **Godot (godot_based_autoware_planning_simulator)** を ego 車両の物理モデルとして利用できる。ego 車両の運動は Godot の `VehicleBody3D` / `VehicleWheel3D` による車両ダイナミクスで計算され、状態は ROS 2 トピック経由で `scenario_simulator_v2` にフィードバックされる。

本ドキュメントは、**Godot 統合時の起動シーケンスと実装の内部構造** を説明する。Godot 本体（`x2_godot` 以下）の変更を必要とせず、`scenario_simulator_v2` 側のみで起動順序を整合させる仕組みを中心に扱う。

## システム構成

```
+-------------------------------+        +------------------------------+
| scenario_simulator_v2         |        | Godot (godot_based_autoware_ |
|                               |        |         planning_simulator)  |
|  - simple_sensor_simulator    | <----> |  - VehicleBody3D             |
|    * EgoEntitySimulation      | rosbridge |  - ros_bridge.gd           |
|    * SimModelExternal         | (ws:9090) |  - lanelet_mesh_builder.gd |
|  - openscenario_interpreter   |        |                              |
|    * FieldOperatorApplication | <----> |                              |
|      (AD-API 経由で engage)    |        |                              |
|  - scenario_test_runner       |        |                              |
|    * lanelet_bridge_node.py   | <----> |                              |
+-------------------------------+        +------------------------------+
           ^                                        ^
           |                                        |
           +------- ROS 2 Topics / Services --------+
           +---- Autoware Launch (map_loader 等) ---+
```

### 主要コンポーネント

| コンポーネント | 役割 |
|---|---|
| `lanelet_bridge_node.py` | `/map/vector_map` (LaneletMapBin) を受信し、Godot が読めるバッチ形式の JSON として配信する |
| `ego_entity_simulation.cpp` (EXTERNAL モード) | Godot 側の `/localization/kinematic_state` を受け取り、`SimModelExternal` を通じて scenario_simulator_v2 の ego 状態を更新する。Godot が ready になった時点で `/initialpose3d` を publish して spawn 位置を通知する |
| `openscenario_interpreter` + `concealer::FieldOperatorApplication` | シナリオ (`.xosc`) の `Storyboard.Init` 完了後、AD-API (`/api/operation_mode/*`, `/api/external/set/engage` など) を呼び出して Autoware を engage する |
| `rosbridge_websocket` | Godot ↔ ROS 2 間の WebSocket ブリッジ (ポート 9090) |
| `scenario_test_runner.launch.py` | 上記プロセスを並列起動する |

## 責務分担

Godot 統合における engage / 初期化の責務分担:

| 処理 | 責務 | 経路 |
|---|---|---|
| **Ego 初期姿勢を Godot に通知** | `EgoEntitySimulation` (EXTERNAL) | `/initialpose3d` トピック (Godot はこれを見て ego を spawn) |
| **Autoware の localization 初期化** | `openscenario_interpreter` → `FieldOperatorApplication::initialize()` | `/api/localization/initialize` サービス |
| **ルート計画** | `openscenario_interpreter` (AcquirePositionAction) → `plan()` | `/api/routing/set_route_points` サービス |
| **operation_mode 遷移（STOP → AUTONOMOUS）** | `openscenario_interpreter` → `FieldOperatorApplication::engage()` | `/api/operation_mode/enable_autoware_control`, `/api/operation_mode/change_to_autonomous` |
| **engage**（`/autoware/engage → /vehicle/engage` の relay でGodot に通知） | `FieldOperatorApplication::engage()` + `autoware_to_vehicle_engage_relay` | `/api/external/set/engage` サービス → `/autoware/engage` → `/vehicle/engage` |
| **Godot のギア遷移（MANUAL → AUTONOMOUS）** | Godot 内部 (`ros_bridge.gd`): `/vehicle/engage = true` 受信で制御モード切替 | `/vehicle/status/control_mode = AUTONOMOUS` の publish |
| **Autoware `is_autoware_control_enabled` セット** | Autoware `command_mode_decider`: `/vehicle/status/control_mode = AUTONOMOUS` を受けて自動的にセット | `OperationModeState.is_autoware_control_enabled = true` |
| **Gear 遷移（PARK → DRIVE）** | Autoware の `shift_decider` が `/autoware/state == DRIVING` かつ `control_cmd.velocity > 0.01` を検知して自動出力 | `/control/command/gear_cmd` トピック |

> **重要**: `FieldOperatorApplication::engage()` は `/api/external/set/engage` のみを呼ぶ。最終的に `operation_mode_transition_manager/compatibility.cpp` が `/autoware/engage` を publish するが、Godot が期待する `/vehicle/engage` は別トピックのため、relay ノードが必要。

## 起動シーケンス

EXTERNAL モードでは、Godot が ego 車両として機能するまでに複数のデータフローと初期化ステップが必要となる。

### 起動シーケンス全体像

```
[Autoware map_loader]
        │
        ▼
/map/vector_map (LaneletMapBin, transient_local)
        │
        ▼
[lanelet_bridge_node.py]  ← Autoware のマップを購読
        │
        ├── batches を準備（ポリゴン・線・信号等を JSON 化）
        │
        ▼
/godot/lanelet_batch_count, /godot/lanelet_batch (Service)
        │
[Godot]  ← rosbridge 経由で Service コール
        │   （rosbridge/lanelet_bridge と並列起動、
        │    接続失敗時は ros_bridge.gd / lanelet_map.gd が 3 秒間隔で自動リトライ）
        │
        ├── viewer_offset 設定 → 全バッチ取得 → mesh (コリジョン含む) 構築
        ├── lanelet_map._built = true
        │
        ▼
/localization/kinematic_state の publish 開始
        │   （Godot は _built == true かつ WebSocket 接続済みの場合のみ publish する）
        │
        ▼
[EgoEntitySimulation::onKinematicState()]
        │
        └── 初受信をワンショットで /initialpose3d を publish
                （Godot 側は initialpose3d を受けて ego を scenario 開始位置に配置）
        │
        ▼
[openscenario_interpreter]  ← Storyboard.Init 完了を検知
        │
        └── FieldOperatorApplication::engage() を呼ぶ
                │
                └── /api/external/set/engage
                        → /autoware/engage (compatibility.cpp が publish)
        │
        ▼
[autoware_to_vehicle_engage_relay]  ← /autoware/engage を購読
        │
        └── /vehicle/engage = true を publish
        │
        ▼
[Godot]  ← /vehicle/engage 受信 → 制御モードを AUTONOMOUS に切替
        │
        └── /vehicle/status/control_mode = AUTONOMOUS を publish
        │
        ▼
[Autoware command_mode_decider]  ← AUTONOMOUS を検知
        │
        └── is_autoware_control_enabled = true をセット
        │
        ▼
[Autoware shift_decider]  ← /autoware/state == DRIVING かつ control_cmd.velocity > 0.01
        │
        └── /control/command/gear_cmd = DRIVE を発行
        │
        ▼
[Godot]  ← DRIVE ギア + control_cmd を受けて走行開始
```

### Godot 準備完了シグナルとしての `/localization/kinematic_state`

`EgoEntitySimulation::onKinematicState()` は、`/localization/kinematic_state` の**初受信**をトリガに `/initialpose3d` を 1 回 publish する。この 1 つの信号で以下全てが保証される：

| 保証内容 | Godot 側根拠 |
|---|---|
| WebSocket 接続済み | `ros_bridge.gd` の `_publish_all()` は `WebSocketPeer.STATE_OPEN` 時のみ走る |
| `car` ノード存在 | `main.gd` 起動直後の `_setup_car()` で `add_child(car)` |
| mesh (コリジョン) 構築完了 | `_publish_all()` の前提 `map_ready = lanelet_map._built == true` |
| viewer_offset 有効 | `_built == true` は `_start_fetch()` の前提 `viewer_offset_valid == true` を内包 |
| 全 lanelet バッチ取得完了 | `_built == true` は `_fetched_batches >= _total_batches` を内包 |

したがって `/godot/lanelet_bridge_ready` のような補助シグナルや `map → viewer` TF の個別監視、settle time の経験値待機は不要で、1 トピックだけで判定が確実・簡潔になる。

### engage は openscenario_interpreter に任せる

EXTERNAL モードにおける engage 処理は `openscenario_interpreter` が担当する。具体的には `openscenario_interpreter.cpp` の `on_activate` で `Storyboard.Init` 完了後に `isAutoware()` な ego ごとに `NonStandardOperation::engage()` を呼び、それが `concealer::FieldOperatorApplication::engage()` (`external/concealer/src/field_operator_application.cpp:280`) に到達する。

`FieldOperatorApplication::engage()` は `/api/external/set/engage` サービスのみを呼ぶ。このサービスは `external_api_adaptor` → `operation_mode_transition_manager/compatibility.cpp` の経路を経て `/autoware/engage = true` を publish する。

### `/vehicle/engage` relay ノードが必要な理由

Godot は [`ros_bridge.gd`] で `/vehicle/engage` トピックを購読し、これを受けて制御モードを MANUAL → AUTONOMOUS に切り替え、`/vehicle/status/control_mode = AUTONOMOUS` を publish する。

しかし現行 Autoware では `/vehicle/engage` を publish する経路が失われている（`awapi_awiv_adapter` の legacy relay は `/awapi/autoware/put/engage` を入力とするが、新 AD-API 経路はここに publish しない）。`/autoware/engage` のみが出力される。

そのため `scenario_test_runner.launch.py` の Godot 分岐内に `topic_tools::relay` ノードを追加し、`/autoware/engage` → `/vehicle/engage` を転送している：

```
/autoware/engage (Autoware) → [autoware_to_vehicle_engage_relay] → /vehicle/engage (Godot)
```

これにより Godot が engage を正しく受信し、`/vehicle/status/control_mode = AUTONOMOUS` を publish するようになる。Autoware の `command_mode_decider` はこれを受けて `is_autoware_control_enabled = true` をセットし、`vehicle_cmd_gate` や `shift_decider` が DRIVE を出せるようになる。

### ギア遷移（PARK → DRIVE）が成立する条件

Godot は起動直後 `GearCommand::PARK (22)` を初期ギアとして保持し、Autoware からの `/control/command/gear_cmd` を受けて切り替える。**gear_cmd が DRIVE にならない限り Godot は前進しない**。

#### shift_decider のロジック

`autoware_shift_decider` (`control/autoware_shift_decider/src/autoware_shift_decider.cpp:62-69`) は以下の条件のときのみ DRIVE を出力する：

```cpp
static constexpr double vel_threshold = 0.01;  // to prevent chattering
if (autoware_state_->state == AutowareState::DRIVING) {
    if (control_cmd_->longitudinal.velocity > vel_threshold) {
        shift_cmd_.command = GearCommand::DRIVE;   // ← ここが DRIVE 出力
    } else if (control_cmd_->longitudinal.velocity < -vel_threshold) {
        shift_cmd_.command = GearCommand::REVERSE;
    } else {
        shift_cmd_.command = prev_shift_command;   // ← それ以外は前回値を維持
    }
}
```

つまり DRIVE に遷移するには次の 2 条件が**同時に**満たされている必要がある：

1. `/autoware/state == DRIVING`（engage 完了後に遷移）
2. `/control/command/control_cmd.longitudinal.velocity > 0.01 m/s`

条件 2 は trajectory follower (MPC) が追跡する `/planning/trajectory` の目標速度に依存する。

#### diffusion planner のスロースタートと stop_point_fixer のデッドロック

diffusion planner は ego が静止中、`v[0] ≈ 0.037 m/s` から始まり時間とともに緩やかに加速する軌道を生成する（実測: v ≥ 0.25 m/s に達するまでに 1.1〜2.9 s かかる）。

`autoware_trajectory_modifier` の `stop_point_fixer` プラグイン (`planning/autoware_trajectory_modifier/src/stop_point_fixer.cpp`) は、ego が停止中かつ以下の条件を満たすとき軌道全体を ego 位置での停止点に置き換える：

```cpp
// is_trajectory_modification_required() (stop_point_fixer.cpp:70-82)
bool StopPointFixer::is_trajectory_modification_required(...)
{
    if (utils::is_ego_vehicle_moving(current_odometry->twist, params_.velocity_threshold))
        return false;  // ego が動いていれば不要
    return is_stop_point_close_to_ego(traj_points) || is_long_stop_trajectory(traj_points);
}

// is_long_stop_trajectory() (stop_point_fixer.cpp:41-59)
bool StopPointFixer::is_long_stop_trajectory(...) const
{
    for (const auto & point : traj_points) {
        const auto t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
        if (t > params_.min_stop_duration)           // ① 時刻チェック（先に評価）
            return true;
        if (point.longitudinal_velocity_mps > params_.velocity_threshold)  // ② 速度チェック
            return false;
    }
    return true;
}
```

パラメータ (`trajectory_modifier.param.yaml`):

```yaml
stop_point_fixer:
  velocity_threshold: 0.25     # [m/s]  "停止" とみなす速度の閾値
  min_stop_duration: 1.0       # [s]    これ以上停止が続く軌道を強制停止とみなす
```

`is_long_stop_trajectory` の評価順序がデッドロックの根本原因だった。ループは各点について **① 時刻チェックを先に、② 速度チェックを後に** 行う。diffusion planner の軌道では `t=1.0 s` まで全点が `v < 0.25 m/s` のため、`t=1.1 s` の点に到達した瞬間に ① が発火して `return true` となり、② の速度チェック（`v=0.275 m/s` に達するのは `t=1.4 s`）が評価される前に軌道が停止点に置き換えられていた。

結果として：

```
diffusion planner: 80pts, v[0]=0.037 m/s
        ↓ stop_point_fixer が発火
/planning/trajectory: 3pts, v=0.000 m/s
        ↓ MPC が追跡
/control/command/control_cmd: velocity=0.000
        ↓ shift_decider が評価
/control/command/gear_cmd: PARK (22) のまま
        ↓
Godot: 前進しない → ego 静止 → stop_point_fixer が再び発火 → デッドロック
```

#### `set_engage_speed: true` による解決

`autoware_trajectory_optimizer` の `TrajectoryVelocityOptimizer` プラグイン (`planning/autoware_trajectory_optimizer/src/trajectory_velocity_optimizer.cpp:83-93`) は、`set_engage_speed: true` のとき ego 速度が `target_pull_out_speed_mps` 未満であれば軌道の速度プロファイル全体を下限クランプする：

```cpp
// trajectory_velocity_optimizer.cpp:83-93
auto initial_motion_speed =
    (current_speed > target_pull_out_speed_mps) ? current_speed : target_pull_out_speed_mps;
auto initial_motion_acc = (current_speed > target_pull_out_speed_mps)
                            ? current_linear_acceleration
                            : target_pull_out_acc_mps2;

if (velocity_params_.set_engage_speed && (current_speed < target_pull_out_speed_mps)) {
    trajectory_velocity_optimizer_utils::clamp_velocities(
        traj_points, static_cast<float>(initial_motion_speed),
        static_cast<float>(initial_motion_acc));
}
```

パラメータ (`trajectory_optimizer_plugins/trajectory_velocity_optimizer.param.yaml`):

```yaml
set_engage_speed: true
target_pull_out_speed_mps: 1.0   # [m/s]
target_pull_out_acc_mps2: 1.0    # [m/s²]
```

ego 静止中 (`current_speed ≈ 0`) は `initial_motion_speed = 1.0 m/s` となり、diffusion planner が生成した軌道の全点速度が 1.0 m/s 以上にクランプされる。これにより：

1. `stop_point_fixer` の `is_long_stop_trajectory`: 軌道の `v[0] = 1.0 m/s >> velocity_threshold (0.25)` → ② 速度チェックが ① 時刻チェックより先に `return false` → **発火しない**
2. `/planning/trajectory`: 80pts, v[0] ≥ 1.0 m/s の正常な軌道が通過
3. MPC: 目標速度 1.0 m/s を追跡 → `control_cmd.velocity > 0.01`
4. `shift_decider` (`DRIVING` 状態 + `velocity > 0.01`): **DRIVE を出力**
5. Godot: DRIVE ギア受信 → 前進開始

なお `trajectory_velocity_optimizer` は `trajectory_modifier`（stop_point_fixer を含む）の**後段**に位置するため、クランプ後の軌道は stop_point_fixer をバイパスして直接 trajectory_selector へ渡される。

```
diffusion_planner → [trajectory_modifier / stop_point_fixer] → [trajectory_optimizer / set_engage_speed] → trajectory_selector → /planning/trajectory
```

stop_point_fixer は `trajectory_modifier` の出力（`/planning/generator/diffusion_planner/modified_candidate_trajectories`）を受け取る時点では v[0]=0.037 のままであるため、依然として発火しうる。しかし、発火して 3pts 停止軌道を出力した場合も、後段の `trajectory_optimizer` で `set_engage_speed` によるクランプが適用されるため、最終的には 1.0 m/s 以上の軌道が `/planning/trajectory` に届く。

> **補足**: Godot 側でも `/vehicle/status/control_mode` (`AUTONOMOUS`) が publish されている間のみ `_apply_autoware_control()` が呼ばれ、受信した `gear_cmd` と `control_cmd` が車両に適用される。`/vehicle/engage` を受信する前は MANUAL のまま publish し続けるため、engage relay ノードと `set_engage_speed` の両方が揃って初めて走行が成立する。

## プロセス起動順序 (`scenario_test_runner.launch.py`)

`use_godot_sim == True` 分岐では、`rosbridge_websocket` / `lanelet_bridge_node` / Godot の 3 プロセスを**並列に起動**する。Godot 側に以下のリトライ機構が備わっているため、起動順の不確定性は自然に吸収される：

| コンポーネント | リトライ仕様 | 実装位置 |
|---|---|---|
| WebSocket 接続 | `STATE_CLOSED` 検知時に 3 秒間隔で `_connect_to_rosbridge()` を再試行 | `ros_bridge.gd` `_process()` |
| `/godot/lanelet_batch_count` Service | 2 秒で応答タイムアウト → `_fetching=false` → 3 秒間隔で `_start_fetch()` 再呼出 | `lanelet_map.gd` `_process()` |
| batch_count == 0 時 | `"No batches available yet, retrying..."` で再試行ループへ | `lanelet_map.gd` `handle_service_response()` |
| `viewer_offset_valid == false` | フェッチ自体を待機し、有効化時に再開 | `lanelet_map.gd` `_process()` |

そのため Godot が rosbridge / lanelet_bridge より先に起動しても、接続失敗・Service タイムアウトのリトライが収束して正常フローに入る。`TimerAction` 等による起動遅延は、経験値（2 秒）で信頼性が保証されず、環境によってはリトライ頼りになるため導入しない。

## 実装ファイル早見表

| ファイル | 責務 |
|---|---|
| `simulation/simple_sensor_simulator/src/vehicle_simulation/ego_entity_simulation.cpp` | EXTERNAL モードの Godot state 取り込み・`/initialpose3d` ワンショット publish |
| `simulation/simple_sensor_simulator/include/simple_sensor_simulator/vehicle_simulation/ego_entity_simulation.hpp` | 上記の型宣言と private メンバ |
| `external/concealer/src/field_operator_application.cpp` | AD-API 経由の engage シーケンス実装（既存、EXTERNAL 用に追加不要） |
| `openscenario/openscenario_interpreter/src/openscenario_interpreter.cpp` | `Storyboard.Init` 完了後に ego ごとに engage を呼ぶ（既存） |
| `test_runner/scenario_test_runner/scenario_test_runner/lanelet_bridge_node.py` | vector_map → Godot 用バッチ JSON 配信、`/tf_static` → `/tf` の relay |
| `test_runner/scenario_test_runner/launch/scenario_test_runner.launch.py` | rosbridge / lanelet_bridge / Godot の並列起動、`/autoware/engage → /vehicle/engage` relay ノードの起動 |
| `src/autoware/launcher/autoware_launch/config/planning/neural_net_planner/trajectory_optimizer_plugins/trajectory_velocity_optimizer.param.yaml` | `set_engage_speed: true` — engage 時の軌道速度下限クランプ（PARK→DRIVE 遷移に必須） |

### `ego_entity_simulation.cpp` のメソッド分割

EXTERNAL モードのコールバック・ロジックは責務ごとに private メソッドへ分割されている：

| メソッド | 呼び出し元 | 役割 |
|---|---|---|
| `initializeExternalMode()` | コンストラクタ | EXTERNAL 専用の subscription・`/initialpose3d` publisher・initial pose メッセージの初期化 |
| `onKinematicState()` | `ego_odometry_sub_` callback | Godot からの odometry を `SimModelExternal` に反映、初受信時にワンショットで `/initialpose3d` を publish |

engage 系のメソッド（`startEngageSequence`, `startGearBootstrap`）は削除された。engage は `openscenario_interpreter` の責務となる。

## トラブルシューティング

### ego が動かない（gear_cmd が PARK のまま）

以下の順に確認する：

1. **`/vehicle/engage` が届いているか**。mcap で `/vehicle/engage` メッセージが 0 件なら、`autoware_to_vehicle_engage_relay` が起動していないか `topic_tools` パッケージが未インストール。`ros2 node list` で relay ノードが見えるか確認。

2. **`/vehicle/status/control_mode` が AUTONOMOUS に切り替わっているか**。`/vehicle/engage` が届いても mode=4 (MANUAL) のままなら Godot 側の engage 受信ロジックの問題。

3. **`is_autoware_control_enabled` が true になっているか**。mcap で `/api/operation_mode/state.is_autoware_control_enabled` が false のままなら Godot が AUTONOMOUS を publish していない（上記 2 が原因）。

4. **`/planning/trajectory` の速度が 0 になっていないか**。mcap で `/planning/trajectory.points[0].longitudinal_velocity_mps` が 0 なら `stop_point_fixer` が発火している可能性がある。`trajectory_velocity_optimizer.param.yaml` の `set_engage_speed: true` が設定されているか確認。

5. **`/control/command/control_cmd.longitudinal.velocity > 0.01` になっているか**。`/planning/trajectory` が正常でも control_cmd が 0 なら、`vehicle_cmd_gate` の `is_engaged_` が false（engage が vehicle_cmd_gate に届いていない）か、MPC のパラメータ問題。

6. **Godot 側で `VehicleWheel3D` が地面に接地していないか**。spawn 位置が mesh の端に近い、あるいは地面の z と `initial_pose.z` に差がある場合、DRIVE ギアでも車輪が空転して前進しない。

### `Waiting for first /localization/kinematic_state from Godot ...` が長く続く

Godot の `lanelet_map._built` が `true` に到達していない。以下を順に確認：

- Godot コンソールで `[LaneletMap] Batch X/Y fetched` が最後のバッチまで進んでいるか
- `ROS bridge: connected to ws://...` が出ているか（未接続なら rosbridge_websocket が立ち上がっていない）
- `[LaneletMap] No batches available yet, retrying...` が出続けている場合は `lanelet_bridge_node` がまだ `/map/vector_map` を受信していない（Autoware の map_loader が未起動）

## 検証手順

```bash
cd /home/kotaroyoshimoto/workspace/x2_godot
colcon build --symlink-install --packages-up-to simple_sensor_simulator scenario_test_runner
source install/setup.bash

ros2 launch scenario_test_runner scenario_test_runner.launch.py \
    scenario:=/path/to/scenario.xosc \
    architecture_type:=awf/universe/20250130 \
    vehicle_model:=<model>_godot \
    record:=true
```

### 期待される `rosout` 出力順序

```
[ego_entity_simulation] Waiting for first /localization/kinematic_state from Godot (signals map/mesh built and ros_bridge ready). initial_pose: x=... y=... z=...
[ego_entity_simulation] First /localization/kinematic_state received; Godot is ready. Published /initialpose3d; engage will be driven by openscenario_interpreter.
(以降は openscenario_interpreter / concealer 側のログ)
[concealer] /api/operation_mode/enable_autoware_control ...
[concealer] /api/operation_mode/change_to_autonomous ...
[concealer] /api/external/set/engage ...
```

順序が大きく前後したり、いずれかのログが出ない場合は起動シーケンスが破綻している。
