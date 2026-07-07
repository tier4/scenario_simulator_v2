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

import ctypes
import os
import tempfile

import rclpy.serialization
import rosbag2_py

from autoware_map_msgs.msg import LaneletMapBin
from autoware_planning_msgs.msg import Trajectory
from pathlib import Path
from .rosbag_merger import find_driving_timestamp

import lanelet2  # noqa: F401 — needed for loadRobust
from lanelet2.io import Origin, loadRobust
from lanelet2.projection import UtmProjector

# Try to load Autoware lanelet2 extension for custom regulatory elements.
for _prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(":"):
    _candidate = os.path.join(_prefix, "lib", "libautoware_lanelet2_extension_lib.so")
    if os.path.isfile(_candidate):
        try:
            ctypes.CDLL(_candidate, mode=ctypes.RTLD_GLOBAL)
        except OSError:
            pass
        break

MAP_TOPIC = "/map/vector_map"
TRAJECTORY_TOPIC = "/planning/trajectory"
DOWNSAMPLE_INTERVAL_NS = 200_000_000  # 200ms → ~5Hz
MARKING_TYPES = {"line_thin", "line_thick", "stop_line"}



def extract_map_data(bag_dir):
    """Read /map/vector_map from bag and extract 2D geometry."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir)),
        rosbag2_py.ConverterOptions("", ""),
    )
    has_map = any(t.name == MAP_TOPIC for t in reader.get_all_topics_and_types())
    if not has_map:
        return {"lanelets": [], "road_markings": []}

    reader.set_filter(rosbag2_py.StorageFilter(topics=[MAP_TOPIC]))
    if not reader.has_next():
        return {"lanelets": [], "road_markings": []}

    _, data, _ = reader.read_next()
    msg = rclpy.serialization.deserialize_message(data, LaneletMapBin)
    lanelet_map = _deserialize_lanelet_map(bytes(msg.data))
    if lanelet_map is None:
        return {"lanelets": [], "road_markings": []}

    return {
        "lanelets": _extract_lanelets(lanelet_map),
        "road_markings": _extract_road_markings(lanelet_map),
    }


def extract_trajectories(bag_dir):
    """Read /planning/trajectory from bag, return downsampled time-series."""
    driving_ts = find_driving_timestamp(bag_dir)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir)),
        rosbag2_py.ConverterOptions("", ""),
    )
    has_topic = any(t.name == TRAJECTORY_TOPIC for t in reader.get_all_topics_and_types())
    if not has_topic:
        return []

    reader.set_filter(rosbag2_py.StorageFilter(topics=[TRAJECTORY_TOPIC]))

    trajectories = []
    last_ts = 0

    while reader.has_next():
        _, data, ts = reader.read_next()
        if driving_ts is not None and ts < driving_ts:
            continue
        if ts - last_ts < DOWNSAMPLE_INTERVAL_NS:
            continue
        last_ts = ts

        traj = rclpy.serialization.deserialize_message(data, Trajectory)
        if len(traj.points) < 2:
            continue

        points = [
            [round(p.pose.position.x, 2), round(p.pose.position.y, 2),
             round(p.longitudinal_velocity_mps, 2)]
            for p in traj.points
        ]
        base_ts = driving_ts if driving_ts is not None else ts
        relative_time = round((ts - base_ts) / 1e9, 3)
        trajectories.append({"t": relative_time, "points": points})

    return trajectories


def _deserialize_lanelet_map(binary_data):
    fd, tmp_path = tempfile.mkstemp(suffix=".bin")
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(binary_data)
        projector = UtmProjector(Origin(0, 0))
        lanelet_map, _ = loadRobust(tmp_path, projector)
        return lanelet_map
    except Exception:
        return None
    finally:
        os.unlink(tmp_path)


def _extract_lanelets(lanelet_map):
    lanelets = []
    for ll in lanelet_map.laneletLayer:
        left = [[round(p.x, 2), round(p.y, 2)] for p in ll.leftBound]
        right = [[round(p.x, 2), round(p.y, 2)] for p in ll.rightBound]
        if len(left) < 2 or len(right) < 2:
            continue
        lanelets.append({"left": left, "right": right})
    return lanelets


def _extract_road_markings(lanelet_map):
    markings = []
    for ls in lanelet_map.lineStringLayer:
        try:
            t = str(ls.attributes["type"])
        except Exception:
            continue
        if t not in MARKING_TYPES:
            continue
        pts = [[round(p.x, 2), round(p.y, 2)] for p in ls]
        if len(pts) < 2:
            continue
        markings.append({"points": pts, "type": t})
    return markings


if __name__ == "__main__":
    import json
    import sys

    bag_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    if bag_dir is None:
        print("Usage: python3 rosbag_reader.py <bag_dir>")
        sys.exit(1)

    print(f"Extracting map from {bag_dir} ...")
    map_data = extract_map_data(bag_dir)
    print(f"  lanelets: {len(map_data['lanelets'])}, road_markings: {len(map_data['road_markings'])}")

    print(f"Extracting trajectories from {bag_dir} ...")
    trajs = extract_trajectories(bag_dir)
    print(f"  frames: {len(trajs)}")
    if trajs:
        print(f"  time range: {trajs[0]['t']:.1f}s - {trajs[-1]['t']:.1f}s")
        print(f"  points per frame: {len(trajs[0]['points'])}")

    data = {
        "map": map_data,
        "models": [{"index": 0, "name": "sample", "trajectories": trajs}],
    }
    out = bag_dir.parent / "report_data.json"
    out.write_text(json.dumps(data))
    print(f"Wrote {out} ({out.stat().st_size / 1024:.0f} KB)")
