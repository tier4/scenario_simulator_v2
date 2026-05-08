#!/usr/bin/env python3
"""フル mcap から三方比較に必要なトピックだけを抽出して lite/*.mcap を生成する.

Usage:
    python3 make_lite.py --kind real --input <full.mcap> --output lite/real.lite.mcap
    python3 make_lite.py --kind sim  --input <full.mcap> --output lite/sim_godot.lite.mcap
    python3 make_lite.py --kind sim  --input <full.mcap> --output lite/sim_normal.lite.mcap
"""

import argparse
from pathlib import Path

from mcap.reader import make_reader
from mcap.writer import Writer

TOPICS: dict[str, set[str]] = {
    "real": {
        "/system/operation_mode/state",
        "/vehicle/status/velocity_status",
        "/vehicle/status/steering_status",
        "/sub/localization/kinematic_state",
        "/sub/localization/acceleration",
        "/sub/control/command/control_cmd",
        # DiffusionPlanner出力軌跡（シムとの直接比較用）
        "/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/output/trajectory",
        # 追跡物体（社会的コンテキスト有無の確認用）
        "/perception/object_recognition/tracking/objects",
        # 最終プランニング軌跡（optimizer後段の出力を確認）
        "/planning/trajectory",
    },
    "sim": {
        "/system/operation_mode/state",
        "/vehicle/status/velocity_status",
        "/vehicle/status/steering_status",
        "/localization/kinematic_state",
        "/localization/acceleration",
        "/control/trajectory_follower/control_cmd",
        # post-gate制御指令（実機 /sub/control/command/control_cmd と同一段での比較用）
        "/control/command/control_cmd",
        # DiffusionPlanner出力軌跡（速度プロファイル分析用）
        "/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/output/trajectory",
        # 交通信号状態（DiffusionPlannerへの入力トピック）
        "/perception/traffic_light_recognition/traffic_signals",
    },
}


def filter_mcap(input_path: Path, output_path: Path, topics: set[str]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(input_path, "rb") as fin, open(output_path, "wb") as fout:
        reader = make_reader(fin)
        writer = Writer(fout)
        writer.start(profile="", library="make_lite")

        schema_ids: dict[int, int] = {}   # 旧 schema_id → 新 schema_id
        channel_ids: dict[int, int] = {}  # 旧 channel_id → 新 channel_id

        for schema, channel, message in reader.iter_messages(topics=list(topics)):
            if schema is not None and schema.id not in schema_ids:
                schema_ids[schema.id] = writer.register_schema(
                    name=schema.name,
                    encoding=schema.encoding,
                    data=schema.data,
                )

            if channel.id not in channel_ids:
                new_schema_id = schema_ids.get(schema.id, 0) if schema else 0
                channel_ids[channel.id] = writer.register_channel(
                    topic=channel.topic,
                    message_encoding=channel.message_encoding,
                    schema_id=new_schema_id,
                    metadata=channel.metadata,
                )

            writer.add_message(
                channel_id=channel_ids[channel.id],
                log_time=message.log_time,
                data=message.data,
                publish_time=message.publish_time,
                sequence=message.sequence,
            )

        writer.finish()

    size_mb = output_path.stat().st_size / 1024 / 1024
    print(f"  書き込み完了: {output_path} ({size_mb:.1f} MB)")


def main() -> None:
    parser = argparse.ArgumentParser(description="mcap トピックフィルタ — lite 版を生成")
    parser.add_argument("--kind", choices=["real", "sim"], required=True,
                        help="ログの種別 (real=実機, sim=シミュレータ)")
    parser.add_argument("--input", required=True, type=Path,
                        help="入力フル mcap ファイルパス")
    parser.add_argument("--output", required=True, type=Path,
                        help="出力 lite mcap ファイルパス")
    args = parser.parse_args()

    if not args.input.exists():
        parser.error(f"入力ファイルが見つかりません: {args.input}")

    topics = TOPICS[args.kind]
    print(f"種別  : {args.kind}")
    print(f"入力  : {args.input} ({args.input.stat().st_size / 1024 / 1024:.0f} MB)")
    print(f"トピック: {sorted(topics)}")
    filter_mcap(args.input, args.output, topics)


if __name__ == "__main__":
    main()
