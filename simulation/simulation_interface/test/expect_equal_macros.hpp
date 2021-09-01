// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#define EXPECT_HEADER_EQ(msg, proto)                            \
  EXPECT_STREQ(msg.frame_id.c_str(), proto.frame_id().c_str()); \
  EXPECT_EQ(msg.stamp.sec, proto.stamp().sec());                \
  EXPECT_EQ(msg.stamp.nanosec, proto.stamp().nanosec());

/**
 * @brief Expect equal macros for builtin_interfaces
 */
#define EXPECT_TIME_EQ(msg, proto)         \
  EXPECT_EQ(msg.nanosec, proto.nanosec()); \
  EXPECT_EQ(msg.sec, proto.sec());

#define EXPECT_DURATION_EQ(msg, proto)     \
  EXPECT_EQ(msg.nanosec, proto.nanosec()); \
  EXPECT_EQ(msg.sec, proto.sec());

/**
 * @brief Expect equal macro for rosgraph_msgs
 */
#define EXPECT_CLOCK_EQ(msg, proto) EXPECT_TIME_EQ(msg.clock, proto.clock());

/**
 * @brief Expect equal macros for geometry_msgs.
 */

#define EXPECT_POINT_EQ(msg, proto)   \
  EXPECT_DOUBLE_EQ(msg.x, proto.x()); \
  EXPECT_DOUBLE_EQ(msg.y, proto.y()); \
  EXPECT_DOUBLE_EQ(msg.z, proto.z());

#define EXPECT_QUATERNION_EQ(msg, proto) \
  EXPECT_DOUBLE_EQ(msg.x, proto.x());    \
  EXPECT_DOUBLE_EQ(msg.y, proto.y());    \
  EXPECT_DOUBLE_EQ(msg.z, proto.z());    \
  EXPECT_DOUBLE_EQ(msg.w, proto.w());

#define EXPECT_POSE_EQ(msg, proto)                            \
  EXPECT_QUATERNION_EQ(msg.orientation, proto.orientation()); \
  EXPECT_POINT_EQ(msg.position, proto.position());

#define EXPECT_VECTOR3_EQ(msg, proto) \
  EXPECT_DOUBLE_EQ(msg.x, proto.x()); \
  EXPECT_DOUBLE_EQ(msg.y, proto.y()); \
  EXPECT_DOUBLE_EQ(msg.z, proto.z());

#define EXPECT_TWIST_EQ(msg, proto)              \
  EXPECT_VECTOR3_EQ(msg.linear, proto.linear()); \
  EXPECT_VECTOR3_EQ(msg.angular, proto.angular());

#define EXPECT_ACCEL_EQ(msg, proto)              \
  EXPECT_VECTOR3_EQ(msg.linear, proto.linear()); \
  EXPECT_VECTOR3_EQ(msg.angular, proto.angular());

/**
 * @brief Expect equal macros for openscenario_msgs.
 */

#define EXPECT_PERFORMANCE_EQ(msg, proto)                           \
  EXPECT_DOUBLE_EQ(msg.max_speed, proto.max_speed());               \
  EXPECT_DOUBLE_EQ(msg.max_acceleration, proto.max_acceleration()); \
  EXPECT_DOUBLE_EQ(msg.max_deceleration, proto.max_deceleration());

#define EXPECT_AXLE_EQ(msg, proto)                              \
  EXPECT_DOUBLE_EQ(msg.max_steering, proto.max_steering());     \
  EXPECT_DOUBLE_EQ(msg.wheel_diameter, proto.wheel_diameter()); \
  EXPECT_DOUBLE_EQ(msg.track_width, proto.track_width());       \
  EXPECT_DOUBLE_EQ(msg.position_x, proto.position_x());         \
  EXPECT_DOUBLE_EQ(msg.position_z, proto.position_z());

#define EXPECT_AXLES_EQ(msg, proto)                  \
  EXPECT_AXLE_EQ(msg.front_axle, proto.front_axle()) \
  EXPECT_AXLE_EQ(msg.rear_axle, proto.rear_axle());

#define EXPECT_BOUNDING_BOX_EQ(msg, proto)     \
  EXPECT_POINT_EQ(msg.center, proto.center()); \
  EXPECT_VECTOR3_EQ(msg.dimensions, proto.dimensions());

#define EXPECT_VEHICLE_PARAMETERS_EQ(msg, proto)                                \
  EXPECT_STREQ(msg.name.c_str(), proto.name().c_str());                         \
  EXPECT_STREQ(msg.vehicle_category.c_str(), proto.vehicle_category().c_str()); \
  EXPECT_BOUNDING_BOX_EQ(msg.bounding_box, proto.bounding_box());               \
  EXPECT_PERFORMANCE_EQ(msg.performance, proto.performance());                  \
  EXPECT_AXLES_EQ(msg.axles, proto.axles());

#define EXPECT_PEDESTRIAN_PARAMETERS_EQ(msg, proto)                                   \
  EXPECT_STREQ(msg.name.c_str(), proto.name().c_str());                               \
  EXPECT_STREQ(msg.pedestrian_category.c_str(), proto.pedestrian_category().c_str()); \
  EXPECT_BOUNDING_BOX_EQ(msg.bounding_box, proto.bounding_box());

#define EXPECT_MISC_OBJECT_PARAMETERS_EQ(msg, proto)                                    \
  EXPECT_STREQ(msg.name.c_str(), proto.name().c_str());                                 \
  EXPECT_STREQ(msg.misc_object_category.c_str(), proto.misc_object_category().c_str()); \
  EXPECT_BOUNDING_BOX_EQ(msg.bounding_box, proto.bounding_box());

#define EXPECT_ACTION_STATUS_EQ(msg, proto)                                 \
  EXPECT_STREQ(msg.current_action.c_str(), proto.current_action().c_str()); \
  EXPECT_TWIST_EQ(msg.twist, proto.twist());                                \
  EXPECT_ACCEL_EQ(msg.accel, proto.accel());

#define EXPECT_LANELET_POSE_EQ(msg, proto)       \
  EXPECT_EQ(msg.lanelet_id, proto.lanelet_id()); \
  EXPECT_DOUBLE_EQ(msg.s, proto.s());            \
  EXPECT_DOUBLE_EQ(msg.offset, proto.offset());  \
  EXPECT_VECTOR3_EQ(msg.rpy, proto.rpy());

#define EXPECT_ENTITY_STATUS_EQ(msg, proto)                          \
  EXPECT_DOUBLE_EQ(msg.time, proto.time());                          \
  EXPECT_STREQ(msg.name.c_str(), proto.name().c_str());              \
  EXPECT_BOUNDING_BOX_EQ(msg.bounding_box, proto.bounding_box());    \
  EXPECT_ACTION_STATUS_EQ(msg.action_status, proto.action_status()); \
  EXPECT_POSE_EQ(msg.pose, proto.pose());                            \
  EXPECT_LANELET_POSE_EQ(msg.lanelet_pose, proto.lanelet_pose());    \
  EXPECT_EQ(msg.lanelet_pose_valid, proto.lanelet_pose_valid());

/**
 * @brief Expect equal macros for autoware related messages.
 */

#define EXPECT_CONTROL_COMMAND_EQ(msg, proto)                                     \
  EXPECT_DOUBLE_EQ(msg.velocity, proto.velocity());                               \
  EXPECT_DOUBLE_EQ(msg.steering_angle_velocity, proto.steering_angle_velocity()); \
  EXPECT_DOUBLE_EQ(msg.steering_angle, proto.steering_angle());                   \
  EXPECT_DOUBLE_EQ(msg.acceleration, proto.acceleration());

#define EXPECT_VEHICLE_COMMAND_EQ(msg, proto)              \
  EXPECT_CONTROL_COMMAND_EQ(msg.control, proto.control()); \
  EXPECT_EQ(msg.shift.data, proto.shift().data());         \
  EXPECT_EQ(msg.emergency, proto.emergency());             \
  EXPECT_HEADER_EQ(msg.header, proto.header());

#endif  // SIMULATION_INTERFACE__TEST__EXPECT_EQUAL_MACROS_HPP_