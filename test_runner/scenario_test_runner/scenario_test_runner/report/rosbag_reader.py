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
from autoware_perception_msgs.msg import PredictedObjects
from autoware_planning_msgs.msg import Trajectory
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import SteeringReport, TurnIndicatorsCommand, VelocityReport
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from traffic_simulator_msgs.msg import EntityStatusWithTrajectoryArray
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
VELOCITY_TOPIC = "/vehicle/status/velocity_status"
ACCEL_TOPIC = "/localization/acceleration"
STEERING_TOPIC = "/vehicle/status/steering_status"
KINEMATIC_TOPIC = "/localization/kinematic_state"
CONTROL_CMD_TOPIC = "/control/command/control_cmd"
TURN_CMD_TOPIC = "/control/command/turn_indicators_cmd"
ENTITY_STATUS_TOPIC = "/simulation/entity/status"
PERCEIVED_OBJECTS_TOPIC = "/perception/object_recognition/objects"
ENTITY_TYPE_EGO = 0
DOWNSAMPLE_INTERVAL_NS = 200_000_000  # 200ms → ~5Hz
RATE_HZ = 10.0
MARKING_TYPES = {"line_thin", "line_thick", "stop_line"}



_SUBTYPE_NAMES = {0: "unknown", 1: "car", 2: "truck", 3: "bus", 4: "trailer",
                  5: "motorcycle", 6: "bicycle", 7: "pedestrian"}


def extract_entities(bag_dir):
    """Extract non-ego entity trajectories and ego bbox from entity/status.

    Returns (entities, ego_bbox) where ego_bbox is
    {length, width, height} or None.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir)),
        rosbag2_py.ConverterOptions("", ""),
    )
    has_topic = any(t.name == ENTITY_STATUS_TOPIC
                    for t in reader.get_all_topics_and_types())
    if not has_topic:
        return [], None

    reader.set_filter(rosbag2_py.StorageFilter(topics=[ENTITY_STATUS_TOPIC]))

    driving_ts = find_driving_timestamp(bag_dir)
    entities = {}
    ego_bbox = None
    base_ts = None
    last_sample_ts = {}

    while reader.has_next():
        _, data, ts = reader.read_next()
        if driving_ts is not None and ts < driving_ts:
            continue
        if base_ts is None:
            base_ts = driving_ts if driving_ts is not None else ts

        msg = rclpy.serialization.deserialize_message(
            data, EntityStatusWithTrajectoryArray)
        t_sec = (ts - base_ts) / 1e9

        for entry in msg.data:
            status = entry.status
            if status.type.type == ENTITY_TYPE_EGO:
                if ego_bbox is None:
                    dim = status.bounding_box.dimensions
                    ego_bbox = {
                        "length": round(dim.x, 2),
                        "width": round(dim.y, 2),
                        "height": round(dim.z, 2),
                    }
                continue
            name = status.name
            if name not in entities:
                dim = status.bounding_box.dimensions
                entities[name] = {
                    "name": name,
                    "subtype": _SUBTYPE_NAMES.get(status.subtype.value, "unknown"),
                    "bbox": {
                        "length": round(dim.x, 2),
                        "width": round(dim.y, 2),
                        "height": round(dim.z, 2),
                    },
                    "positions": [],
                }
                last_sample_ts[name] = 0

            if ts - last_sample_ts[name] < DOWNSAMPLE_INTERVAL_NS:
                continue
            last_sample_ts[name] = ts

            p = status.pose.position
            o = status.pose.orientation
            yaw = math.atan2(
                2 * (o.w * o.z + o.x * o.y),
                1 - 2 * (o.y * o.y + o.z * o.z))
            entities[name]["positions"].append({
                "t": round(t_sec, 3),
                "x": round(p.x, 2),
                "y": round(p.y, 2),
                "yaw": round(yaw, 3),
            })

    return list(entities.values()), ego_bbox


def extract_predicted_objects(bag_dir, topic):
    """Extract object snapshots from a PredictedObjects topic.

    Returns a list of snapshots: [{t, objects: [{x, y, yaw, label, length, width}]}].
    Downsampled to ~5 Hz.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir)),
        rosbag2_py.ConverterOptions("", ""),
    )
    has_topic = any(t.name == topic for t in reader.get_all_topics_and_types())
    if not has_topic:
        return []

    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    driving_ts = find_driving_timestamp(bag_dir)
    base_ts = None
    snapshots = []

    while reader.has_next():
        _, data, ts = reader.read_next()
        if driving_ts is not None and ts < driving_ts:
            continue
        if base_ts is None:
            base_ts = driving_ts if driving_ts is not None else ts

        msg = rclpy.serialization.deserialize_message(data, PredictedObjects)
        t_sec = (ts - base_ts) / 1e9
        objs = []
        for obj in msg.objects:
            p = obj.kinematics.initial_pose_with_covariance.pose.position
            o = obj.kinematics.initial_pose_with_covariance.pose.orientation
            yaw = math.atan2(
                2 * (o.w * o.z + o.x * o.y),
                1 - 2 * (o.y * o.y + o.z * o.z))
            label = 0
            if obj.classification:
                label = obj.classification[0].label
            dim = obj.shape.dimensions
            objs.append({
                "x": round(p.x, 2),
                "y": round(p.y, 2),
                "yaw": round(yaw, 3),
                "label": _SUBTYPE_NAMES.get(label, "unknown"),
                "length": round(dim.x, 2),
                "width": round(dim.y, 2),
            })
        snapshots.append({"t": round(t_sec, 3), "objects": objs})

    return snapshots


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


def _extract_velocity_timeseries(bag_dir, driving_ts):
    """Extract longitudinal velocity [m/s] from /vehicle/status/velocity_status."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir)),
        rosbag2_py.ConverterOptions("", ""),
    )
    has_topic = any(t.name == VELOCITY_TOPIC for t in reader.get_all_topics_and_types())
    if not has_topic:
        return []

    reader.set_filter(rosbag2_py.StorageFilter(topics=[VELOCITY_TOPIC]))
    rows = []
    base_ts = None
    while reader.has_next():
        _, data, ts = reader.read_next()
        if driving_ts is not None and ts < driving_ts:
            continue
        if base_ts is None:
            base_ts = driving_ts if driving_ts is not None else ts
        msg = rclpy.serialization.deserialize_message(data, VelocityReport)
        rows.append(((ts - base_ts) / 1e9, msg.longitudinal_velocity))
    return rows


def _extract_accel_timeseries(bag_dir, driving_ts):
    """Extract longitudinal acceleration [m/s²] from /localization/acceleration."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir)),
        rosbag2_py.ConverterOptions("", ""),
    )
    has_topic = any(t.name == ACCEL_TOPIC for t in reader.get_all_topics_and_types())
    if not has_topic:
        return []

    reader.set_filter(rosbag2_py.StorageFilter(topics=[ACCEL_TOPIC]))
    rows = []
    base_ts = None
    while reader.has_next():
        _, data, ts = reader.read_next()
        if driving_ts is not None and ts < driving_ts:
            continue
        if base_ts is None:
            base_ts = driving_ts if driving_ts is not None else ts
        msg = rclpy.serialization.deserialize_message(data, AccelWithCovarianceStamped)
        rows.append(((ts - base_ts) / 1e9, msg.accel.accel.linear.x))
    return rows


def _extract_steering_timeseries(bag_dir, driving_ts):
    """Extract steering tire angle [rad] from /vehicle/status/steering_status."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir)),
        rosbag2_py.ConverterOptions("", ""),
    )
    has_topic = any(t.name == STEERING_TOPIC for t in reader.get_all_topics_and_types())
    if not has_topic:
        return []

    reader.set_filter(rosbag2_py.StorageFilter(topics=[STEERING_TOPIC]))
    rows = []
    base_ts = None
    while reader.has_next():
        _, data, ts = reader.read_next()
        if driving_ts is not None and ts < driving_ts:
            continue
        if base_ts is None:
            base_ts = driving_ts if driving_ts is not None else ts
        msg = rclpy.serialization.deserialize_message(data, SteeringReport)
        rows.append(((ts - base_ts) / 1e9, msg.steering_tire_angle))
    return rows


def _extract_kinematic_state(bag_dir, driving_ts):
    """Extract ego pose (t, x, y, yaw) and yaw rate from /localization/kinematic_state."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir)),
        rosbag2_py.ConverterOptions("", ""),
    )
    has_topic = any(t.name == KINEMATIC_TOPIC for t in reader.get_all_topics_and_types())
    if not has_topic:
        return [], []

    reader.set_filter(rosbag2_py.StorageFilter(topics=[KINEMATIC_TOPIC]))
    ego_rows = []
    wz_rows = []
    base_ts = None
    last_ts = 0
    while reader.has_next():
        _, data, ts = reader.read_next()
        if driving_ts is not None and ts < driving_ts:
            continue
        if base_ts is None:
            base_ts = driving_ts if driving_ts is not None else ts
        msg = rclpy.serialization.deserialize_message(data, Odometry)
        t_sec = (ts - base_ts) / 1e9
        wz_rows.append((t_sec, msg.twist.twist.angular.z))
        if ts - last_ts < DOWNSAMPLE_INTERVAL_NS:
            continue
        last_ts = ts
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        ego_rows.append((t_sec, p.x, p.y, yaw))
    return ego_rows, wz_rows


def _extract_control_cmd_timeseries(bag_dir, driving_ts):
    """Extract control command (vel, accel, steer) from /control/command/control_cmd."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir)),
        rosbag2_py.ConverterOptions("", ""),
    )
    has_topic = any(t.name == CONTROL_CMD_TOPIC for t in reader.get_all_topics_and_types())
    if not has_topic:
        return [], [], []

    reader.set_filter(rosbag2_py.StorageFilter(topics=[CONTROL_CMD_TOPIC]))
    cmd_vel_rows = []
    cmd_accel_rows = []
    cmd_steer_rows = []
    base_ts = None
    while reader.has_next():
        _, data, ts = reader.read_next()
        if driving_ts is not None and ts < driving_ts:
            continue
        if base_ts is None:
            base_ts = driving_ts if driving_ts is not None else ts
        msg = rclpy.serialization.deserialize_message(data, Control)
        t_sec = (ts - base_ts) / 1e9
        cmd_vel_rows.append((t_sec, msg.longitudinal.velocity))
        cmd_accel_rows.append((t_sec, msg.longitudinal.acceleration))
        cmd_steer_rows.append((t_sec, msg.lateral.steering_tire_angle))
    return cmd_vel_rows, cmd_accel_rows, cmd_steer_rows


def _extract_turn_cmd_timeseries(bag_dir, driving_ts):
    """Extract turn indicator command from /control/command/turn_indicators_cmd."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir)),
        rosbag2_py.ConverterOptions("", ""),
    )
    has_topic = any(t.name == TURN_CMD_TOPIC for t in reader.get_all_topics_and_types())
    if not has_topic:
        return []

    reader.set_filter(rosbag2_py.StorageFilter(topics=[TURN_CMD_TOPIC]))
    rows = []
    base_ts = None
    while reader.has_next():
        _, data, ts = reader.read_next()
        if driving_ts is not None and ts < driving_ts:
            continue
        if base_ts is None:
            base_ts = driving_ts if driving_ts is not None else ts
        msg = rclpy.serialization.deserialize_message(data, TurnIndicatorsCommand)
        rows.append(((ts - base_ts) / 1e9, float(msg.command)))
    return rows


def _extract_planned_frames(bag_dir, driving_ts):
    """Extract planned trajectory snapshots from /planning/trajectory."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir)),
        rosbag2_py.ConverterOptions("", ""),
    )
    has_topic = any(t.name == TRAJECTORY_TOPIC for t in reader.get_all_topics_and_types())
    if not has_topic:
        return []

    reader.set_filter(rosbag2_py.StorageFilter(topics=[TRAJECTORY_TOPIC]))
    planned_frames = []
    last_planned_t = -1.0

    while reader.has_next():
        _, data, ts = reader.read_next()
        if driving_ts is not None and ts < driving_ts:
            continue
        traj = rclpy.serialization.deserialize_message(data, Trajectory)
        if len(traj.points) < 2:
            continue
        base_ts = driving_ts if driving_ts is not None else ts
        t_sec = (ts - base_ts) / 1e9
        if t_sec - last_planned_t >= 0.5:
            last_planned_t = t_sec
            points = [
                [round(p.pose.position.x, 2), round(p.pose.position.y, 2),
                 round(p.longitudinal_velocity_mps, 2)]
                for p in traj.points[::3]
            ]
            planned_frames.append({"t": round(t_sec, 3), "points": points})
    return planned_frames


def _extract_ego_timeseries(bag_dir):
    """Extract ego state from dedicated status/localization topics."""
    driving_ts = find_driving_timestamp(bag_dir)

    ego_ts, wz_ts = _extract_kinematic_state(bag_dir, driving_ts)
    if not ego_ts:
        return None, [], [], [], [], [], [], [], [], []

    planned_frames = _extract_planned_frames(bag_dir, driving_ts)
    vel_ts = _extract_velocity_timeseries(bag_dir, driving_ts)
    accel_ts = _extract_accel_timeseries(bag_dir, driving_ts)
    steer_ts = _extract_steering_timeseries(bag_dir, driving_ts)
    cmd_vel_ts, cmd_accel_ts, cmd_steer_ts = _extract_control_cmd_timeseries(bag_dir, driving_ts)
    turn_cmd_ts = _extract_turn_cmd_timeseries(bag_dir, driving_ts)

    return (ego_ts, planned_frames, vel_ts, accel_ts, steer_ts, wz_ts,
            cmd_vel_ts, cmd_accel_ts, cmd_steer_ts, turn_cmd_ts)


def _interp_ts(t_grid, ts_rows):
    """Interpolate a list of (t, value) onto t_grid. Returns zeros if empty."""
    if not ts_rows:
        return np.zeros_like(t_grid)
    data = np.array(ts_rows, dtype=float)
    return np.interp(t_grid, data[:, 0], data[:, 1])


def _nearest_ts(t_grid, ts_rows):
    """Sample-and-hold (nearest-backward) for discrete-valued timeseries."""
    if not ts_rows:
        return np.zeros_like(t_grid)
    data = np.array(ts_rows, dtype=float)
    idx = np.searchsorted(data[:, 0], t_grid, side="right") - 1
    idx = np.clip(idx, 0, len(data) - 1)
    return data[idx, 1]


def _resample_to_grid(ego_ts, vel_ts, accel_ts, steer_ts, wz_ts,
                      cmd_vel_ts, cmd_accel_ts, cmd_steer_ts, turn_cmd_ts):
    """Resample ego timeseries onto a uniform time grid."""
    if not ego_ts:
        return None

    data = np.array(ego_ts, dtype=float)
    t_raw = data[:, 0]
    x_raw, y_raw = data[:, 1], data[:, 2]
    yaw_raw = data[:, 3]

    t_max = t_raw[-1]
    n = int(np.floor(t_max * RATE_HZ)) + 1
    t_grid = np.arange(n) / RATE_HZ

    x = np.interp(t_grid, t_raw, x_raw)
    y = np.interp(t_grid, t_raw, y_raw)
    yaw = np.interp(t_grid, t_raw, np.unwrap(yaw_raw))
    v = _interp_ts(t_grid, vel_ts)
    accel = _interp_ts(t_grid, accel_ts)
    steer = _interp_ts(t_grid, steer_ts)
    yaw_rate = _interp_ts(t_grid, wz_ts)
    cmd_vel = _interp_ts(t_grid, cmd_vel_ts)
    cmd_accel = _interp_ts(t_grid, cmd_accel_ts)
    cmd_steer = _interp_ts(t_grid, cmd_steer_ts)
    turn_cmd = _nearest_ts(t_grid, turn_cmd_ts)

    n_valid = int(np.searchsorted(t_grid, t_raw[-1], side="right"))
    v[n_valid:] = 0.0

    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    s_total = float(s[n_valid - 1]) if n_valid > 0 else 0.0

    def _to_list(arr, dp=2):
        out = [round(float(v), dp) for v in arr[:n_valid]]
        out.extend([None] * (n - n_valid))
        return out

    def _has_data(arr):
        return n_valid > 1 and np.max(np.abs(arr[:n_valid])) > 1e-8

    ch = {}
    if vel_ts:
        ch["velocity"] = _to_list(v)
    if accel_ts:
        ch["accel"] = _to_list(accel)
    if _has_data(yaw_rate):
        ch["heading_rate"] = _to_list(yaw_rate, 4)
    if _has_data(steer):
        ch["steer"] = _to_list(np.degrees(steer))
    if cmd_vel_ts:
        ch["cmd_velocity"] = _to_list(cmd_vel)
    if cmd_accel_ts:
        ch["cmd_accel"] = _to_list(cmd_accel)
    if cmd_steer_ts:
        ch["cmd_steer"] = _to_list(np.degrees(cmd_steer))
    if turn_cmd_ts:
        ch["turn_cmd"] = [int(v) for v in turn_cmd[:n_valid]] + [None] * (n - n_valid)

    return {
        "x": [round(float(v), 2) for v in x],
        "y": [round(float(v), 2) for v in y],
        "yaw": [round(float(v), 3) for v in yaw],
        "v": [round(float(v), 2) for v in v],
        "s": [round(float(v), 2) for v in s],
        "n_valid": n_valid,
        "s_total": round(s_total, 2),
        "ch": ch,
    }


def extract_trajectories(bag_dir):
    """Read trajectory, steering, kinematic, and control command data from bag."""
    (ego_ts, planned_frames, vel_ts, accel_ts, steer_ts, wz_ts,
     cmd_vel_ts, cmd_accel_ts, cmd_steer_ts, turn_cmd_ts,
     ) = _extract_ego_timeseries(bag_dir)
    if not ego_ts:
        return {"x": [], "y": [], "yaw": [], "v": [], "s": [],
                "n_valid": 0, "s_total": 0.0, "planned": [], "ch": {
                    "velocity": [], "accel": [], "heading_rate": [], "steer": []}}

    result = _resample_to_grid(
        ego_ts, vel_ts, accel_ts, steer_ts, wz_ts,
        cmd_vel_ts, cmd_accel_ts, cmd_steer_ts, turn_cmd_ts)
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
