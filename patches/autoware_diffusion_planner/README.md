# autoware_diffusion_planner lockstep パッチ

`0001-lockstep-service-mode.patch` は `autoware_diffusion_planner` に
ロックステップサービスモードを追加する最小差分です。

## 変更概要

- `lockstep_service_mode` パラメータ（デフォルト: `false`）を追加
  - `true` のとき周期タイマーを無効化し、`PlanTrajectory` サービス
    （`/planning/diffusion_planner/srv/plan_trajectory`）で同期実行
  - `false` のとき従来どおり `on_timer` で動作（後方互換性維持）
- `on_timer` を `run_planning_cycle` + 薄いラッパーにリファクタリング
- lockstep モードではルートを polling でなくコールバック購読で受け取る
  （並列負荷時の取りこぼし対策）

## 適用方法

```bash
cd <autoware_universe_workspace>/src/autoware/universe
git am < /path/to/this/0001-lockstep-service-mode.patch
```

または差分のみ適用する場合:

```bash
git apply < /path/to/this/0001-lockstep-service-mode.patch
```

## 依存関係

パッチ適用後、`package.xml` に `diffusion_planner_lockstep_msgs` が追加されます。
`diffusion_planner_lockstep_msgs` パッケージ（本リポの
`external/diffusion_planner_lockstep_msgs/`）を先にビルドしてください。

## 対象バージョン

パッチは autoware_universe `21cd3ddd84` をベースに生成されています。
他のバージョンに適用する場合は `git apply --3way` か手動マージを推奨します。
