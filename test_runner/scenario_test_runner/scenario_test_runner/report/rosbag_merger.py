#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2020 TIER IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import os
import shutil
import tempfile

import rclpy.serialization
import rosbag2_py

from autoware_planning_msgs.msg import Trajectory
from geometry_msgs.msg import Point
from pathlib import Path
from shutil import rmtree
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


# --- Trajectory -> MarkerArray conversion ---
# Ribbon geometry and color logic based on WebAutoCITools:
# frontend/scenario_editor/src/services/three-js/viewer-objects/trajectory.ts

RIBBON_HALF_WIDTH = 1.125  # 2.25 / 2
RIBBON_OPACITY = 0.6


def velocity_color(velocity):
    vel = abs(velocity)
    if vel <= 0.0:
        r, g, b = 1.0, 0.0, 0.0
    elif vel <= 0.5:
        ratio = vel / 0.5
        r = 1.0
        g = ratio
        b = 0.0
    elif vel <= 1.0:
        ratio = (vel - 0.5) / 0.5
        r = 1.0 - ratio
        g = 1.0
        b = 0.0
    else:
        r, g, b = 0.0, 1.0, 0.0
    return ColorRGBA(r=r, g=g, b=b, a=RIBBON_OPACITY)


def quaternion_to_yaw(o):
    siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
    cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
    return math.atan2(siny_cosp, cosy_cosp)


def convert_trajectory_to_marker_array(data, model_name=""):
    traj = rclpy.serialization.deserialize_message(data, Trajectory)
    marker_array = MarkerArray()

    if len(traj.points) < 2:
        return rclpy.serialization.serialize_message(marker_array)

    marker = Marker()
    marker.header = traj.header
    marker.ns = model_name
    marker.id = 0
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.a = 1.0
    marker.pose.orientation.w = 1.0

    prev_left = None
    prev_right = None
    prev_color = None

    for pt in traj.points:
        p = pt.pose.position
        yaw = quaternion_to_yaw(pt.pose.orientation)
        t = yaw + math.pi / 2.0
        dx = RIBBON_HALF_WIDTH * math.cos(t)
        dy = RIBBON_HALF_WIDTH * math.sin(t)

        left = Point(x=p.x - dx, y=p.y - dy, z=p.z)
        right = Point(x=p.x + dx, y=p.y + dy, z=p.z)
        color = velocity_color(pt.longitudinal_velocity_mps)

        if prev_left is not None:
            marker.points.extend([left, prev_left, right,
                                  prev_right, right, prev_left])
            marker.colors.extend([color, prev_color, color,
                                  prev_color, color, prev_color])

        prev_left = left
        prev_right = right
        prev_color = color

    marker_array.markers.append(marker)
    return rclpy.serialization.serialize_message(marker_array)


# --- Rosbag merge ---

AUTOWARE_STATE_TOPIC = "/autoware/state"
AUTOWARE_STATE_DRIVING = 5

# (topic_name) -> list of (topic_infix, output_type_string, converter_or_None)
# topic_infix is inserted between /models/model_N and the original topic name.
# converter_or_None: None = pass-through (raw bytes).
TOPIC_OUTPUTS = {
    "/planning/trajectory": [
        ("", "autoware_planning_msgs/msg/Trajectory", None),
        ("/markers", "visualization_msgs/msg/MarkerArray", convert_trajectory_to_marker_array),
        ("/auto", "autoware_auto_planning_msgs/msg/Trajectory", None),
    ],
}


def find_driving_timestamp(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir)),
        rosbag2_py.ConverterOptions("", ""),
    )
    has_topic = any(
        t.name == AUTOWARE_STATE_TOPIC for t in reader.get_all_topics_and_types()
    )
    if not has_topic:
        return None
    reader.set_filter(rosbag2_py.StorageFilter(topics=[AUTOWARE_STATE_TOPIC]))
    while reader.has_next():
        _, data, ts = reader.read_next()
        # CDR: 4-byte encap + 8-byte stamp + 1-byte state
        if len(data) >= 13 and data[12] == AUTOWARE_STATE_DRIVING:
            return ts
    return None


def detect_storage_id(bag_dir):
    for f in bag_dir.iterdir():
        if f.suffix == ".mcap":
            return "mcap"
        elif f.suffix == ".db3":
            return "sqlite3"
    return "mcap"


def merge_single_rosbag(target_bag_dir, run_bag_dir, run_name, topics, logger):
    run_reader = rosbag2_py.SequentialReader()
    run_reader.open(
        rosbag2_py.StorageOptions(uri=str(run_bag_dir)),
        rosbag2_py.ConverterOptions("", ""),
    )

    topic_type_map = {
        t.name: t for t in run_reader.get_all_topics_and_types() if t.name in topics
    }
    if not topic_type_map:
        return

    run_prefix = f"/runs/{run_name}"
    output_map = {}
    for orig_name, orig_meta in topic_type_map.items():
        outputs = TOPIC_OUTPUTS.get(orig_name)
        if outputs:
            output_map[orig_name] = [
                (f"{run_prefix}{infix}{orig_name}", out_type, converter)
                for infix, out_type, converter in outputs
            ]
        else:
            output_map[orig_name] = [
                (f"{run_prefix}{orig_name}", orig_meta.type, None)
            ]

    target_driving_ts = find_driving_timestamp(target_bag_dir)
    run_driving_ts = find_driving_timestamp(run_bag_dir)
    if target_driving_ts is None or run_driving_ts is None:
        logger.warn(
            f"[Model Compare] DRIVING state not found, skipping run {run_name}"
        )
        return
    ts_offset = target_driving_ts - run_driving_ts

    merged_bag_dir = Path(tempfile.mkdtemp(prefix="merge_"))
    merged_bag_path = merged_bag_dir / "merged"
    try:
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(
                uri=str(merged_bag_path), storage_id=detect_storage_id(target_bag_dir),
            ),
            rosbag2_py.ConverterOptions("", ""),
        )

        target_reader = rosbag2_py.SequentialReader()
        target_reader.open(
            rosbag2_py.StorageOptions(uri=str(target_bag_dir)),
            rosbag2_py.ConverterOptions("", ""),
        )
        for topic_meta in target_reader.get_all_topics_and_types():
            writer.create_topic(topic_meta)
        for orig_name, entries in output_map.items():
            orig_meta = topic_type_map[orig_name]
            for out_name, out_type, _ in entries:
                writer.create_topic(
                    rosbag2_py.TopicMetadata(
                        name=out_name,
                        type=out_type,
                        serialization_format=orig_meta.serialization_format,
                    )
                )

        target_messages = []
        while target_reader.has_next():
            target_messages.append(target_reader.read_next())

        run_reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
        run_messages = []
        while run_reader.has_next():
            topic, data, timestamp = run_reader.read_next()
            adjusted_ts = timestamp + ts_offset
            for out_name, _, converter in output_map[topic]:
                out_data = converter(data, model_name=run_name) if converter else data
                run_messages.append((out_name, out_data, adjusted_ts))

        if not run_messages:
            return

        all_messages = target_messages + run_messages
        all_messages.sort(key=lambda m: m[2])
        for topic, data, timestamp in all_messages:
            writer.write(topic, data, timestamp)
        del writer

        for f in target_bag_dir.iterdir():
            f.unlink()
        for f in merged_bag_path.iterdir():
            try:
                os.rename(str(f), str(target_bag_dir / f.name))
            except OSError:
                shutil.copy2(str(f), str(target_bag_dir / f.name))
                os.remove(str(f))

        out_topics = [n for entries in output_map.values() for n, _, _ in entries]
        logger.info(
            f"[Model Compare] Merged {len(run_messages)} messages from run {run_name} "
            f"into {target_bag_dir.name} ({out_topics})"
        )
    finally:
        rmtree(merged_bag_dir, ignore_errors=True)
