// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SIMULATION_INTERFACE__TEST__EXPECT_EQUAL_MACROS_HPP_
#define SIMULATION_INTERFACE__TEST__EXPECT_EQUAL_MACROS_HPP_

#include <geometry_msgs.pb.h>
#include <gtest/gtest.h>

#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <simulation_interface/conversions.hpp>
#include <string>

/**
 * @brief Expect equal macros for std_msgs.
 */

#define EXPECT_HEADER_EQ(MSG, PROTO)                            \
  EXPECT_STREQ(MSG.frame_id.c_str(), PROTO.frame_id().c_str()); \
  EXPECT_EQ(MSG.stamp.sec, PROTO.stamp().sec());                \
  EXPECT_EQ(MSG.stamp.nanosec, PROTO.stamp().nanosec());

/**
 * @brief Expect equal macros for builtin_interfaces
 */
#define EXPECT_TIME_EQ(MSG, PROTO)         \
  EXPECT_EQ(MSG.nanosec, PROTO.nanosec()); \
  EXPECT_EQ(MSG.sec, PROTO.sec());

#define EXPECT_DURATION_EQ(MSG, PROTO)     \
  EXPECT_EQ(MSG.nanosec, PROTO.nanosec()); \
  EXPECT_EQ(MSG.sec, PROTO.sec());

/**
 * @brief Expect equal macro for rosgraph_msgs
 */
#define EXPECT_CLOCK_EQ(MSG, PROTO) EXPECT_TIME_EQ(MSG.clock, PROTO.clock());

/**
 * @brief Expect equal macros for geometry_msgs.
 */

#define EXPECT_POINT_EQ(MSG, PROTO)   \
  EXPECT_DOUBLE_EQ(MSG.x, PROTO.x()); \
  EXPECT_DOUBLE_EQ(MSG.y, PROTO.y()); \
  EXPECT_DOUBLE_EQ(MSG.z, PROTO.z());

#define EXPECT_QUATERNION_EQ(MSG, PROTO) \
  EXPECT_DOUBLE_EQ(MSG.x, PROTO.x());    \
  EXPECT_DOUBLE_EQ(MSG.y, PROTO.y());    \
  EXPECT_DOUBLE_EQ(MSG.z, PROTO.z());    \
  EXPECT_DOUBLE_EQ(MSG.w, PROTO.w());

#define EXPECT_POSE_EQ(MSG, PROTO)                            \
  EXPECT_QUATERNION_EQ(MSG.orientation, PROTO.orientation()); \
  EXPECT_POINT_EQ(MSG.position, PROTO.position());

#define EXPECT_VECTOR3_EQ(MSG, PROTO) \
  EXPECT_DOUBLE_EQ(MSG.x, PROTO.x()); \
  EXPECT_DOUBLE_EQ(MSG.y, PROTO.y()); \
  EXPECT_DOUBLE_EQ(MSG.z, PROTO.z());

#define EXPECT_TWIST_EQ(MSG, PROTO)              \
  EXPECT_VECTOR3_EQ(MSG.linear, PROTO.linear()); \
  EXPECT_VECTOR3_EQ(MSG.angular, PROTO.angular());

#define EXPECT_ACCEL_EQ(MSG, PROTO)              \
  EXPECT_VECTOR3_EQ(MSG.linear, PROTO.linear()); \
  EXPECT_VECTOR3_EQ(MSG.angular, PROTO.angular());

/**
 * @brief Expect equal macros for traffic_simulator_msgs.
 */

#define EXPECT_PERFORMANCE_EQ(MSG, PROTO)                                     \
  EXPECT_DOUBLE_EQ(MSG.max_speed, PROTO.max_speed());                         \
  EXPECT_DOUBLE_EQ(MSG.max_acceleration, PROTO.max_acceleration());           \
  EXPECT_DOUBLE_EQ(MSG.max_acceleration_rate, PROTO.max_acceleration_rate()); \
  EXPECT_DOUBLE_EQ(MSG.max_deceleration, PROTO.max_deceleration());           \
  EXPECT_DOUBLE_EQ(MSG.max_deceleration_rate, PROTO.max_deceleration_rate());

#define EXPECT_AXLE_EQ(MSG, PROTO)                              \
  EXPECT_DOUBLE_EQ(MSG.max_steering, PROTO.max_steering());     \
  EXPECT_DOUBLE_EQ(MSG.wheel_diameter, PROTO.wheel_diameter()); \
  EXPECT_DOUBLE_EQ(MSG.track_width, PROTO.track_width());       \
  EXPECT_DOUBLE_EQ(MSG.position_x, PROTO.position_x());         \
  EXPECT_DOUBLE_EQ(MSG.position_z, PROTO.position_z());

#define EXPECT_AXLES_EQ(MSG, PROTO)                  \
  EXPECT_AXLE_EQ(MSG.front_axle, PROTO.front_axle()) \
  EXPECT_AXLE_EQ(MSG.rear_axle, PROTO.rear_axle());

#define EXPECT_BOUNDING_BOX_EQ(MSG, PROTO)     \
  EXPECT_POINT_EQ(MSG.center, PROTO.center()); \
  EXPECT_VECTOR3_EQ(MSG.dimensions, PROTO.dimensions());

#define EXPECT_VEHICLE_PARAMETERS_EQ(MSG, PROTO)                  \
  EXPECT_STREQ(MSG.name.c_str(), PROTO.name().c_str());           \
  EXPECT_BOUNDING_BOX_EQ(MSG.bounding_box, PROTO.bounding_box()); \
  EXPECT_PERFORMANCE_EQ(MSG.performance, PROTO.performance());    \
  EXPECT_AXLES_EQ(MSG.axles, PROTO.axles());

#define EXPECT_PEDESTRIAN_PARAMETERS_EQ(MSG, PROTO)     \
  EXPECT_STREQ(MSG.name.c_str(), PROTO.name().c_str()); \
  EXPECT_BOUNDING_BOX_EQ(MSG.bounding_box, PROTO.bounding_box());

#define EXPECT_MISC_OBJECT_PARAMETERS_EQ(MSG, PROTO)    \
  EXPECT_STREQ(MSG.name.c_str(), PROTO.name().c_str()); \
  EXPECT_BOUNDING_BOX_EQ(MSG.bounding_box, PROTO.bounding_box());

#define EXPECT_ACTION_STATUS_EQ(MSG, PROTO)                                 \
  EXPECT_STREQ(MSG.current_action.c_str(), PROTO.current_action().c_str()); \
  EXPECT_TWIST_EQ(MSG.twist, PROTO.twist());                                \
  EXPECT_ACCEL_EQ(MSG.accel, PROTO.accel());                                \
  EXPECT_DOUBLE_EQ(MSG.linear_jerk, PROTO.linear_jerk());

#define EXPECT_LANELET_POSE_EQ(MSG, PROTO)       \
  EXPECT_EQ(MSG.lanelet_id, PROTO.lanelet_id()); \
  EXPECT_DOUBLE_EQ(MSG.s, PROTO.s());            \
  EXPECT_DOUBLE_EQ(MSG.offset, PROTO.offset());  \
  EXPECT_VECTOR3_EQ(MSG.rpy, PROTO.rpy());

#define EXPECT_ENTITY_STATUS_EQ(MSG, PROTO)                          \
  EXPECT_DOUBLE_EQ(MSG.time, PROTO.time());                          \
  EXPECT_STREQ(MSG.name.c_str(), PROTO.name().c_str());              \
  EXPECT_BOUNDING_BOX_EQ(MSG.bounding_box, PROTO.bounding_box());    \
  EXPECT_ACTION_STATUS_EQ(MSG.action_status, PROTO.action_status()); \
  EXPECT_POSE_EQ(MSG.pose, PROTO.pose());                            \
  EXPECT_LANELET_POSE_EQ(MSG.lanelet_pose, PROTO.lanelet_pose());    \
  EXPECT_EQ(MSG.lanelet_pose_valid, PROTO.lanelet_pose_valid());

#define EXPECT_SENT_ENTITY_STATUS_EQ(MSG, PROTO)                     \
  EXPECT_DOUBLE_EQ(MSG.time, PROTO.time());                          \
  EXPECT_STREQ(MSG.name.c_str(), PROTO.name().c_str());              \
  EXPECT_ACTION_STATUS_EQ(MSG.action_status, PROTO.action_status()); \
  EXPECT_POSE_EQ(MSG.pose, PROTO.pose());

/**
 * @brief Expect equal macros for autoware related messages.
 */
#define EXPECT_CONTROL_COMMAND_EQ(MSG, PROTO)                                               \
  EXPECT_DOUBLE_EQ(MSG.longitudinal.velocity, PROTO.longitudinal().velocity());             \
  EXPECT_DOUBLE_EQ(MSG.longitudinal.acceleration, PROTO.longitudinal().acceleration());     \
  EXPECT_DOUBLE_EQ(MSG.lateral.steering_tire_angle, PROTO.lateral().steering_tire_angle()); \
  EXPECT_DOUBLE_EQ(                                                                         \
    MSG.lateral.steering_tire_rotation_rate, PROTO.lateral().steering_tire_rotation_rate());

#define EXPECT_VEHICLE_COMMAND_EQ(CONTROL_MSG, GEAR_MSG, EMERGENCY_MSG, PROTO) \
  EXPECT_CONTROL_COMMAND_EQ(CONTROL_MSG, PROTO.control());                     \
  EXPECT_DOUBLE_EQ(GEAR_MSG.command, PROTO.gear_command().data());             \
  EXPECT_DOUBLE_EQ(EMERGENCY_MSG.emergency, PROTO.vehicle_emergency_stamped().emergency());

#endif  // SIMULATION_INTERFACE__TEST__EXPECT_EQUAL_MACROS_HPP_
