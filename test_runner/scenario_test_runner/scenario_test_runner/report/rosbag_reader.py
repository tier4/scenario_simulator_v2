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
import math
import os
import tempfile

import numpy as np
import rclpy.serialization
import rosbag2_py

from autoware_map_msgs.msg import LaneletMapBin
from autoware_planning_msgs.msg import Trajectory
from pathlib import Path
from .rosbag_merger import find_driving_timestamp, quaternion_to_yaw

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
RATE_HZ = 10.0
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


def _extract_ego_timeseries(bag_dir):
    """Extract per-frame ego state from the first point of each trajectory message."""
    driving_ts = find_driving_timestamp(bag_dir)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir)),
        rosbag2_py.ConverterOptions("", ""),
    )
    has_topic = any(t.name == TRAJECTORY_TOPIC for t in reader.get_all_topics_and_types())
    if not has_topic:
        return None, []

    reader.set_filter(rosbag2_py.StorageFilter(topics=[TRAJECTORY_TOPIC]))

    ego_ts = []
    planned_frames = []
    last_ts = 0
    last_planned_t = -1.0

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

        base_ts = driving_ts if driving_ts is not None else ts
        t_sec = (ts - base_ts) / 1e9

        p0 = traj.points[0]
        yaw = quaternion_to_yaw(p0.pose.orientation)
        ego_ts.append((
            t_sec,
            p0.pose.position.x,
            p0.pose.position.y,
            yaw,
            p0.longitudinal_velocity_mps,
            p0.acceleration_mps2,
            p0.heading_rate_rps,
            p0.front_wheel_angle_rad,
        ))

        if t_sec - last_planned_t >= 0.5:
            last_planned_t = t_sec
            points = [
                [round(p.pose.position.x, 2), round(p.pose.position.y, 2),
                 round(p.longitudinal_velocity_mps, 2)]
                for p in traj.points[::3]
            ]
            planned_frames.append({"t": round(t_sec, 3), "points": points})

    return ego_ts, planned_frames


def _resample_to_grid(ego_ts):
    """Resample ego timeseries onto a uniform time grid."""
    if not ego_ts:
        return None

    data = np.array(ego_ts, dtype=float)
    t_raw = data[:, 0]
    x_raw, y_raw = data[:, 1], data[:, 2]
    yaw_raw = data[:, 3]
    v_raw = data[:, 4]
    accel_raw = data[:, 5]
    hr_raw = data[:, 6]
    steer_raw = data[:, 7]

    t_max = t_raw[-1]
    n = int(np.floor(t_max * RATE_HZ)) + 1
    t_grid = np.arange(n) / RATE_HZ

    x = np.interp(t_grid, t_raw, x_raw)
    y = np.interp(t_grid, t_raw, y_raw)
    yaw = np.interp(t_grid, t_raw, np.unwrap(yaw_raw))
    v = np.interp(t_grid, t_raw, v_raw)
    accel = np.interp(t_grid, t_raw, accel_raw)
    hr = np.interp(t_grid, t_raw, hr_raw)
    steer = np.interp(t_grid, t_raw, steer_raw)

    n_valid = int(np.searchsorted(t_grid, t_raw[-1], side="right"))
    v[n_valid:] = 0.0

    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    s_total = float(s[n_valid - 1]) if n_valid > 0 else 0.0

    def _to_list(arr, dp=2):
        out = [round(float(v), dp) for v in arr[:n_valid]]
        out.extend([None] * (n - n_valid))
        return out

    return {
        "x": [round(float(v), 2) for v in x],
        "y": [round(float(v), 2) for v in y],
        "yaw": [round(float(v), 3) for v in yaw],
        "v": [round(float(v), 2) for v in v],
        "s": [round(float(v), 2) for v in s],
        "n_valid": n_valid,
        "s_total": round(s_total, 2),
        "ch": {
            "velocity": _to_list(v),
            "accel": _to_list(accel),
            "heading_rate": _to_list(hr, 3),
            "steer": _to_list(np.degrees(steer)),
        },
    }


def extract_trajectories(bag_dir):
    """Read /planning/trajectory from bag, return resampled ego data + planned frames."""
    ego_ts, planned_frames = _extract_ego_timeseries(bag_dir)
    if not ego_ts:
        return {"x": [], "y": [], "yaw": [], "v": [], "s": [],
                "n_valid": 0, "s_total": 0.0, "planned": [], "ch": {
                    "velocity": [], "accel": [], "heading_rate": [], "steer": []}}

    result = _resample_to_grid(ego_ts)
    result["planned"] = planned_frames
    return result


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
    traj_data = extract_trajectories(bag_dir)
    n = len(traj_data["x"])
    print(f"  grid points: {n}, n_valid: {traj_data['n_valid']}, "
          f"s_total: {traj_data['s_total']:.1f}m")
    print(f"  planned frames: {len(traj_data['planned'])}")

    data = {
        "rate_hz": RATE_HZ,
        "n": n,
        "map": map_data,
        "models": [{"name": "sample", "color": "hsl(210,70%,55%)", **traj_data}],
    }
    out = bag_dir.parent / "report_data.json"
    out.write_text(json.dumps(data, separators=(",", ":")))
    print(f"Wrote {out} ({out.stat().st_size / 1024:.0f} KB)")
