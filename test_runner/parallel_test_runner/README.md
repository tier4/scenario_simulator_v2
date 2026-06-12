# parallel_test_runner

Web.Auto のシナリオ（suite / vehicle catalog）をローカルで **N並列・高速ロックステップ構成**で実行するテストランナー。

diffusion_planner を同期サービス（`/planning/diffusion_planner/srv/plan_trajectory`）として呼び出し、
openscenario_interpreter を best-effort ループ（wall timer 0ms）で回すことで、
シミュレーション時間を実時間より大幅に速く進める。

## 実測パフォーマンス（RTX 4090 / 32コア、6並列）

| 指標 | MPSなし | `--mps` |
|---|---|---|
| 1ワーカーあたりRTF（起動抜き） | 2.8倍 | 4.1倍 |
| システム全体RTF（起動抜き） | 16.8倍 | **24.8倍** |

### 並列数スケーリング（`--mps` あり）

| `--jobs` | 6 | 8 | 10 | **12** | 16 |
|---|---|---|---|---|---|
| 実行時間 | 3分36秒 | 3分11秒 | 2分59秒 | **2分40秒** | 2分49秒 |
| peak VRAM | - | 8.7 GB | 10.9 GB | 12.1 GB | 17.3 GB |

MPSなしでは6並列でほぼ頭打ちだが、`--mps` ありでは**12並列付近が最速**（16では逆に低下）。
VRAMは約1.0〜1.1GB/ワーカー。判定結果は並列数によらず完全に一致する。

## 前提条件

- ワークスペースがビルド済み（`parallel_test_runner`, `scenario_test_runner`, `simple_sensor_simulator`,
  `openscenario_interpreter`, `diffusion_planner_lockstep_msgs` ほか）
- diffusion_planner のモデル/TensorRTエンジンがビルド済み（パスは `diffusion_planner.param.yaml` の `onnx_model_path` / `args_path` で指定）
- `webauto` CLI が認証済み（suite / catalog の取得に使用。ローカルファイル実行のみなら不要）
- `--mps` を使う場合: `nvidia-cuda-mps-control` が PATH にあること（CUDAドライバ付属）

## 使い方

```bash
# vehicle catalog（所属する全suite）を6並列+MPSで実行
ros2 run parallel_test_runner parallel_test_runner \
  --project-id <project-id> --catalog-id <uuid> \
  --vehicle-model <vehicle>_perfect_tracker --jobs 6 --mps

# suite 単位で実行
ros2 run parallel_test_runner parallel_test_runner \
  --project-id <project-id> --suite-id <uuid> \
  --vehicle-model <vehicle>_perfect_tracker --jobs 6

# ダウンロードのみ / 一覧のみ（実行しない）
ros2 run parallel_test_runner parallel_test_runner \
  --project-id <project-id> --catalog-id <uuid> --pull-only   # manifest.json を作成
ros2 run parallel_test_runner parallel_test_runner \
  --project-id <project-id> --catalog-id <uuid> --list-only   # 一覧表示のみ

# 取得済み manifest から実行（pull と実行の時間を分けたいとき）
ros2 run parallel_test_runner parallel_test_runner \
  --manifest /tmp/parallel_test_runner_assets/manifest.json \
  --vehicle-model <vehicle>_perfect_tracker --jobs 6 --mps

# ローカルのシナリオファイルを直接実行（.yaml / .xosc、繰り返し指定可）
ros2 run parallel_test_runner parallel_test_runner \
  --scenario path/to/scenario.yml --vehicle-model <vehicle>_perfect_tracker
```

## 主なオプション

| オプション | デフォルト | 説明 |
|---|---|---|
| `--jobs N` | 4 | 並列ワーカー数。MPSなしは6でほぼ頭打ち、`--mps` ありは12付近が最速（スケーリング表参照） |
| `--mps` | off | ワーカー実行を NVIDIA MPS デーモンで包む（走行フェーズ約1.5倍）。既存デーモンは再利用、自分で起動した場合のみ終了時に停止 |
| `--vehicle-model` | sample_vehicle_perfect_tracker | `_perfect_tracker` で終わる必要あり |
| `--output-directory` | /tmp/parallel_test_runner | ワーカー出力・マージ済みjunitの出力先 |
| `--work-dir` | /tmp/parallel_test_runner_assets | シナリオ・マップのダウンロード先（キャッシュとして再利用される） |
| `--base-domain-id` | 60 | ワーカー i は ROS_DOMAIN_ID = base + i を使用 |
| `--base-port` | 6000 | ワーカー i は ZMQ port = base + i を使用 |
| `--global-timeout` | 600 | シナリオ本体に許す壁時計秒数 |
| `--startup-margin` | 240 | 起動（Autoware launch・TensorRTロード等）に上乗せする壁時計秒数 |
| `--launch-arg key:=value` | - | scenario_test_runner.launch.py への追加引数（繰り返し可） |

## 出力

```
<output-directory>/
├── expanded/<label>/scenario_*.xosc   # yaml→xosc 展開結果（permutationごと）
├── workers/<NNN>_<label>/
│   ├── launch.log                     # そのケースの全ノードログ
│   └── scenario_test_runner/result.junit.xml
├── nvidia-mps-log/                    # --mps 時のMPSデーモンログ
└── result.junit.xml                   # 全ケースのマージ結果
```

ケースの判定は junit の failures / errors から `PASSED` / `FAILED` /
`MISSING_RESULT`（junitが生成されなかった: ワーカークラッシュやpreprocessor段階の除外）/
`TIMEOUT` に分類され、終了コードは全PASSEDのとき0。

## モジュール構成

| モジュール | 責務 |
|---|---|
| `parallel_test_runner.py` | CLI・ワークフロー全体 |
| `webauto_provider.py` | suite / catalog の解決と `webauto ci scenario pull`、manifest.json の生成・読込 |
| `expansion.py` | yaml → permutationごとの .xosc 展開（純Python） |
| `parallel_runner.py` | 汎用N並列エンジン（ROS_DOMAIN_ID分離・プロセスグループ管理・junitマージ）。起動コマンドは知らない |
| `lockstep_profile.py` | ロックステップ構成のlaunch引数組み立て・ZMQポート割当 |
| `mps.py` | NVIDIA MPS デーモンのライフサイクル管理（contextmanager） |
| `minimal_adapi_stub.py` | 最小構成用 AD API スタブ（launch/minimal_e2e_simulator.launch.xml から使用） |

並列実行はプル型ワークキュー方式で、ワーカー間の同期バリアはない（結合はGPU競合のみ）。

## 既知の注意点

- **CYCLONEDDS_URI が設定されている環境では ROS_LOCALHOST_ONLY を付与しない**
  （インターフェース二重選択で全ノードSIGABRT）。parallel_runner が自動で判定する
- 6並列ではGPU（fp32エンジン）が飽和し、1ワーカーあたりの速度は単発時の約9倍速から2.8〜4.1倍速に低下する。
  さらなる改善候補: FP16エンジン化、共有plannerサーバ+バッチ推論
- ゴール未達のシナリオはシナリオ制限時間（120〜180シム秒）をフルに走るため、
  FAILEDが多いスイートほど実行時間が長くなる
- Web.Auto のシナリオyamlで `maximumExecutionCount` が指数表記される問題は
  openscenario_utility/conversion.py 側で整数化対応済み
