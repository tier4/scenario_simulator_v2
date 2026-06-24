# CARLA で動作させる

`pilot-auto.x2` の `feat/v4.3/e2e` をベースに、 scenario_simulator のみ
本リポジトリの実装を載せ替えて、 CARLA 0.10.0 (Tier4 Odaiba build) を
Autoware の `planning_simulator.launch.xml` から動かす手順。

## 前提

### ワークスペース

- `pilot-auto.x2` の `feat/v4.3/e2e` を checkout 済み
- `src/simulator/scenario_simulator` だけ `temporary/diffusion_planner_carla` ブランチに差し替え

### CARLA バイナリの展開

CARLA 0.10.0 Odaiba build (特別版) を入手し任意のディレクトリに展開
(以下 `<carla>` と表記)。

### CARLA Python wheel のインストール

```
pip install --user <carla>/PythonAPI/carla/dist/carla-0.10.0-cp310-cp310-linux_x86_64.whl
```

### Odaiba 地図の取得

WebAutoCLI で取得

```
webauto map area-map pull --project-id x2_dev --area-map-id 2231
```

展開先 path は
`$HOME/.webauto/simulation/data/map/x2_dev/2231/2231-<timestamp>`
の形式 (例: `2231-20260331083910239532`)。 これを後段の `map_path` に指定する。

## launch の差し替え

`src/autoware/universe/launch/tier4_simulator_launch/launch/simulator.launch.xml`
の `<group if="$(var launch_dummy_vehicle)">` ブロックの中身を以下で置換:

```xml
<group if="$(var launch_dummy_vehicle)">
  <node pkg="scenario_test_runner" exec="carla_bridge.py" name="carla_bridge" output="screen">
    <param name="host" type="str" value="127.0.0.1"/>
    <param name="port" type="int" value="2000"/>
    <param name="hz_rate" type="int" value="30"/>
  </node>
</group>
```

(従来の `simple_planning_simulator` 系の include はすべて削除)

## 起動

### Terminal A: CARLA 本体

```
cd <carla>
./Linux/CarlaUnreal.sh /Game/Carla/Maps/Odaiba \
  -vulkan -prefernvidia -log -game -noraytracing -quality-level=Low --ros2
```

ウィンドウに Odaiba の地形が表示されるまで待つ

### Terminal B: Autoware + carla_bridge

```
source install/setup.bash

ros2 launch autoware_launch planning_simulator.launch.xml \
  vehicle_model:=j6_gen2 \
  sensor_model:=aip_x2_gen2 \
  map_path:=$HOME/.webauto/simulation/data/map/x2_dev/2231/2231-<timestamp>
```

## 動作確認フロー

1. **起動完了** (約 30–40 秒)
   - CARLA: テレポート駅前 に J6 の ego が出現
   - Rviz: 地図 / ego 車体モデルが表示される
   - AutowareStatePanel: **Localization = Initialized**

2. **route 設定**
   - Rvizの視点設定（Views）の「Type」で「TopDownOrtho」を指定して「Zero」をクリックして視点を設定
   - 必要に応じて、**2D Pose Estimate** ツールで地図上をクリックして初期位置を移動
   - Rviz の **2D Goal Pose** ツールで目標地点をクリック
   - 緑線で route が描画される
   - AutowareStatePanel: **Routing = Set**

3. **Engage**
   - AutowareStatePanel で **AutowareControl を ON** (ControlMode = AUTONOMOUS)
   - AutowareStatePanel で **Auto モード** に切り替え (Operation Mode = AUTONOMOUS)
   - ego が trajectory に沿って走り出す

