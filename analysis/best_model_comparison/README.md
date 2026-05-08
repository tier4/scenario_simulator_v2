# best_model_comparison

実機ログ / Godot シム / 通常シム の三方比較プロット・レポート生成ツール。

シナリオ: `x2_dev_teleport_to_miraikan`（x2_dev/2231、テレポート駅→日本科学未来館）

## 依存パッケージ

```bash
pip install mcap mcap-ros2-support numpy pandas matplotlib lxml
```

## ワークフロー

### 初回セットアップ: 実機 lite mcap を生成する（一度のみ）

実機ログは固定なので、初回のみ手動で lite 版を生成する。

```bash
cd analysis/best_model_comparison

# 実機ログ（/sub/* トピック）
python3 make_lite.py --kind real \
  --input /path/to/real/filtered_*.mcap \
  --output lite/real.lite.mcap
```

生成される lite ファイルは合計約 8 MB。`lite/` は `.gitignore` で除外済み。

### 通常運用: シム再実行 + 解析（Makefile）

シミュレータの改修のたびに以下を実行する。
**事前に `source install/setup.bash` 済みのシェルで操作すること。**

```bash
cd analysis/best_model_comparison

# 各シムを個別に再実行（常にシムを起動し直す）
make sim_normal   # 通常シム (j6_gen2) → lite/sim_normal.lite.mcap 更新
make sim_godot    # Godot シム (j6_gen2_godot) → lite/sim_godot.lite.mcap 更新

# 解析レポート生成のみ（シムは再実行しない）
make report

# シム実行 + 解析を一括実行
make all
```

シム実行の流れ: `ros2 launch scenario_test_runner` → `/tmp/scenario_test_runner/` 以下に mcap 記録 → `make_lite.py` で lite 変換 → フル mcap を削除。

利用可能な全ターゲットの確認:

```bash
make help
```

出力先: `comparison/figures/*.{png,pdf}` と `comparison/report.md`（`.gitignore` で除外済み）。

## 出力サンプル（参考値）

| 指標 | 実機 | Godot シム | 通常シム |
|---|---|---|---|
| 完走時間 [s] | 207.9 | 185.7 | 173.0 |
| 平均速度 [m/s] | 2.051 | 5.164 | 5.299 |
| 速度 RMSE [m/s] | 0.1313 | 0.4429 | 0.5887 |
| ステアリング RMSE [deg] | 0.8741 | 1.9241 | 0.3752 |
| 軌跡乖離・平均 [m] | — (基準) | 2.423 | 2.644 |

## 注意

- **Lanelet2 地図**: `~/.webauto/simulation/data/map/x2_dev/2231/.../lanelet2_map.osm` が存在すれば軌跡プロットに地図背景を重ねる。ファイルがない場合は地図なしで自動フォールバックする。
- **実機ログのトピック名**: 実機は `/sub/localization/*`, `/sub/control/*` を使用。シムは `/localization/*`, `/control/trajectory_follower/*`。`make_lite.py` の `--kind` で自動切替。
