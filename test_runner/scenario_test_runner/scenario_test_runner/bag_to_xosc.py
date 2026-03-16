#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2015 TIER IV, Inc. All rights reserved.
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
import tempfile
import argparse
import yaml
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple

import rclpy.serialization
import rosbag2_py
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry

KINEMATIC_STATE_TOPIC = "/localization/kinematic_state"


def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class OdomPose:
    x: float
    y: float
    z: float
    yaw: float


def _odom_bytes_to_pose(data: bytes) -> OdomPose:
    msg = rclpy.serialization.deserialize_message(data, Odometry)
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    return OdomPose(
        x=pos.x,
        y=pos.y,
        z=pos.z,
        yaw=_quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w),
    )


@dataclass
class VehicleInfo:
    wheel_radius: float
    wheel_width: float
    wheel_base: float
    wheel_tread: float
    front_overhang: float
    rear_overhang: float
    left_overhang: float
    right_overhang: float
    vehicle_height: float
    max_steer_angle: float
    vel_lim: float
    vel_rate_lim: float


def _load_ros_parameters(yaml_path: Path) -> dict:
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
        return data["/**"]["ros__parameters"]


def read_vehicle_params(vehicle_model: str) -> VehicleInfo:
    pkg_path = Path(get_package_share_directory(vehicle_model + "_description")) / "config"

    vehicle_info = _load_ros_parameters(pkg_path / "vehicle_info.param.yaml")
    simulator_model = _load_ros_parameters(pkg_path / "simulator_model.param.yaml")

    return VehicleInfo(
        wheel_radius=float(vehicle_info["wheel_radius"]),
        wheel_width=float(vehicle_info["wheel_width"]),
        wheel_base=float(vehicle_info["wheel_base"]),
        wheel_tread=float(vehicle_info["wheel_tread"]),
        front_overhang=float(vehicle_info["front_overhang"]),
        rear_overhang=float(vehicle_info["rear_overhang"]),
        left_overhang=float(vehicle_info["left_overhang"]),
        right_overhang=float(vehicle_info["right_overhang"]),
        vehicle_height=float(vehicle_info["vehicle_height"]),
        max_steer_angle=float(vehicle_info["max_steer_angle"]),
        vel_lim=float(simulator_model["vel_lim"]),
        vel_rate_lim=float(simulator_model["vel_rate_lim"]),
    )


def read_odom_from_bag(
    bag_path: str,
    start_time_s: float = 0.0,
) -> Tuple[OdomPose, OdomPose, float]:
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(Path(bag_path))), rosbag2_py.ConverterOptions("", ""))

    storage_filter = rosbag2_py.StorageFilter(topics=[KINEMATIC_STATE_TOPIC])
    reader.set_filter(storage_filter)

    first_time_ns: Optional[int] = None
    initial_time_ns: Optional[int] = None
    initial_pose: Optional[OdomPose] = None
    last_data: Optional[bytes] = None
    last_time_ns = 0

    while reader.has_next():
        _, data, timestamp_ns = reader.read_next()

        if first_time_ns is None:
            first_time_ns = timestamp_ns

        elapsed_s = (timestamp_ns - first_time_ns) * 1e-9
        if elapsed_s < start_time_s:
            continue

        if initial_pose is None:
            initial_pose = _odom_bytes_to_pose(data)
            initial_time_ns = timestamp_ns

        last_data = data
        last_time_ns = timestamp_ns

    if initial_pose is None:
        raise RuntimeError(
            f"No {KINEMATIC_STATE_TOPIC} messages found in bag "
            f"(start_time={start_time_s}s or later): {bag_path}"
        )

    duration_s = (last_time_ns - initial_time_ns) * 1e-9
    last_pose = _odom_bytes_to_pose(last_data)

    print(
        f"[bag_to_xosc] initial=({initial_pose.x:.2f}, {initial_pose.y:.2f})"
        f"  goal=({last_pose.x:.2f}, {last_pose.y:.2f})"
        f"  duration={duration_s:.1f}s"
    )

    return initial_pose, last_pose, duration_s


# xosc template
_XOSC_TEMPLATE = """\
<?xml version="1.0" encoding="UTF-8"?>
<OpenSCENARIO>
  <FileHeader
    revMajor="1" revMinor="3"
    date="{date}"
    description="Generated from bag replay: {bag_name}"
    author="scenario_test_runner"/>
  <CatalogLocations/>
  <RoadNetwork>
    <LogicFile filepath="{map_path}"/>
  </RoadNetwork>
  <Entities>
    <ScenarioObject name="ego">
      <Vehicle name="{vehicle_model}" vehicleCategory="car">
        <ParameterDeclarations/>
        <Performance maxSpeed="{max_speed:.3f}" maxAcceleration="{max_accel:.3f}" maxDeceleration="{max_decel:.3f}"/>
        <BoundingBox>
          <Center x="{bounding_box_center_x:.3f}" y="{bounding_box_center_y:.3f}" z="{bounding_box_center_z:.3f}"/>
          <Dimensions width="{bounding_box_width:.3f}" length="{bounding_box_length:.3f}" height="{bounding_box_height:.3f}"/>
        </BoundingBox>
        <Axles>
          <FrontAxle
            maxSteering="{front_max_steering:.4f}" wheelDiameter="{wheel_diameter:.4f}" trackWidth="{front_track_width:.4f}"
            positionX="{front_position_x:.5f}" positionZ="{wheel_position_z:.4f}"/>
          <RearAxle
            maxSteering="0.0" wheelDiameter="{wheel_diameter:.4f}" trackWidth="{rear_track_width:.4f}"
            positionX="0.0" positionZ="{wheel_position_z:.4f}"/>
        </Axles>
        <Properties/>
      </Vehicle>
      <ObjectController>
        <Controller name="Autoware">
          <Properties>
            <Property name="isEgo" value="true"/>
          </Properties>
        </Controller>
      </ObjectController>
    </ScenarioObject>
  </Entities>
  <Storyboard>
    <Init>
      <Actions>
        <Private entityRef="ego">
          <PrivateAction>
            <TeleportAction>
              <Position>
                <WorldPosition
                  x="{init_x:.6f}" y="{init_y:.6f}" z="{init_z:.6f}"
                  h="{init_h:.6f}"/>
              </Position>
            </TeleportAction>
          </PrivateAction>
          <PrivateAction>
            <RoutingAction>
              <AcquirePositionAction>
                <Position>
                  <WorldPosition
                    x="{goal_x:.6f}" y="{goal_y:.6f}" z="{goal_z:.6f}"
                    h="{goal_h:.6f}"/>
                </Position>
              </AcquirePositionAction>
            </RoutingAction>
          </PrivateAction>
        </Private>
      </Actions>
    </Init>
    <Story name="">
      <Act name="">
        <ManeuverGroup name="" maximumExecutionCount="1">
          <Actors selectTriggeringEntities="false"/>
          <Maneuver name="">
            <Event name="" priority="overwrite">
              <Action name="">
                <UserDefinedAction>
                  <CustomCommandAction type="exitSuccess"/>
                </UserDefinedAction>
              </Action>
              <StartTrigger>
                <ConditionGroup>
                  <Condition name="" delay="0" conditionEdge="none">
                    <ByValueCondition>
                      <SimulationTimeCondition value="{duration:.3f}" rule="greaterOrEqual"/>
                    </ByValueCondition>
                  </Condition>
                </ConditionGroup>
              </StartTrigger>
            </Event>
          </Maneuver>
        </ManeuverGroup>
        <StartTrigger>
          <ConditionGroup>
            <Condition name="" delay="0" conditionEdge="none">
              <ByValueCondition>
                <SimulationTimeCondition value="0" rule="greaterOrEqual"/>
              </ByValueCondition>
            </Condition>
          </ConditionGroup>
        </StartTrigger>
        <StopTrigger/>
      </Act>
    </Story>
    <StopTrigger/>
  </Storyboard>
</OpenSCENARIO>
"""


def generate_xosc_for_bag(
    bag_path: str,
    map_path: str,
    initial_pose: OdomPose,
    goal_pose: OdomPose,
    duration_s: float,
    vehicle_info: VehicleInfo,
    vehicle_model: str,
    output_path: Optional[Path] = None,
) -> Path:
    bounding_box_length = vehicle_info.front_overhang + vehicle_info.wheel_base + vehicle_info.rear_overhang
    bounding_box_width = vehicle_info.left_overhang + vehicle_info.wheel_tread + vehicle_info.right_overhang
    bounding_box_center_x = (vehicle_info.front_overhang + vehicle_info.wheel_base - vehicle_info.rear_overhang) / 2.0
    bounding_box_center_z = vehicle_info.vehicle_height / 2.0

    content = _XOSC_TEMPLATE.format(
        date=datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
        bag_name=Path(bag_path).name,
        map_path=map_path,
        vehicle_model=vehicle_model,
        max_speed=vehicle_info.vel_lim,
        max_accel=vehicle_info.vel_rate_lim,
        max_decel=vehicle_info.vel_rate_lim,
        bounding_box_center_x=bounding_box_center_x,
        bounding_box_center_y=0.0,
        bounding_box_center_z=bounding_box_center_z,
        bounding_box_width=bounding_box_width,
        bounding_box_length=bounding_box_length,
        bounding_box_height=vehicle_info.vehicle_height,
        front_max_steering=vehicle_info.max_steer_angle,
        wheel_diameter=vehicle_info.wheel_radius * 2.0,
        front_track_width=vehicle_info.wheel_tread,
        front_position_x=vehicle_info.wheel_base,
        wheel_position_z=vehicle_info.wheel_radius,
        rear_track_width=vehicle_info.wheel_tread,
        init_x=initial_pose.x,
        init_y=initial_pose.y,
        init_z=initial_pose.z,
        init_h=initial_pose.yaw,
        goal_x=goal_pose.x,
        goal_y=goal_pose.y,
        goal_z=goal_pose.z,
        goal_h=goal_pose.yaw,
        duration=duration_s,
    )

    if output_path is None:
        fd, temporary_name = tempfile.mkstemp(suffix=".xosc")
        os.close(fd)
        output_path = Path(temporary_name)

    output_path = Path(output_path)
    output_path.write_text(content, encoding="utf-8")

    print(f"[bag_to_xosc] Generated xosc: {output_path}")
    return output_path


def main():
    parser = argparse.ArgumentParser(
        description="Generate a minimal OpenSCENARIO .xosc from a rosbag"
    )
    parser.add_argument("--bag", required=True, type=str, help="rosbag file path or directory path")
    parser.add_argument("--map", required=True, type=str, help="Lanelet2 map path or directory path")
    parser.add_argument("--vehicle-model", required=True, type=str, help="vehicle model name (e.g. j6_gen2, sample_vehicle)")
    parser.add_argument("--output", default=None, type=str, help="output .xosc path (defaults to a temp file under /tmp)")
    parser.add_argument("--start-time", default=0.0, type=float, help="playback start offset from the bag beginning [s]")
    args = parser.parse_args()

    vehicle_info = read_vehicle_params(args.vehicle_model)
    initial_pose, goal_pose, duration_s = read_odom_from_bag(args.bag, args.start_time)
    output_path = Path(args.output) if args.output else None
    xosc_path = generate_xosc_for_bag(
        bag_path=args.bag,
        map_path=args.map,
        initial_pose=initial_pose,
        goal_pose=goal_pose,
        duration_s=duration_s,
        vehicle_info=vehicle_info,
        vehicle_model=args.vehicle_model,
        output_path=output_path,
    )
    print(xosc_path)


if __name__ == "__main__":
    main()
