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

#ifndef SIMPLE_SENSOR_SIMULATOR__TEST__UTILS__EXPECT_EQ_MACROS_HPP_
#define SIMPLE_SENSOR_SIMULATOR__TEST__UTILS__EXPECT_EQ_MACROS_HPP_

#include <gtest/gtest.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#define EXPECT_POINT_EQ(DATA0, DATA1) \
  EXPECT_DOUBLE_EQ(DATA0.x, DATA1.x); \
  EXPECT_DOUBLE_EQ(DATA0.y, DATA1.y); \
  EXPECT_DOUBLE_EQ(DATA0.z, DATA1.z);

#define EXPECT_POINT_NEAR(DATA0, DATA1, TOLERANCE) \
  EXPECT_NEAR(DATA0.x, DATA1.x, TOLERANCE);        \
  EXPECT_NEAR(DATA0.y, DATA1.y, TOLERANCE);        \
  EXPECT_NEAR(DATA0.z, DATA1.z, TOLERANCE);

#define EXPECT_QUATERNION_EQ(DATA0, DATA1) \
  EXPECT_DOUBLE_EQ(DATA0.x, DATA1.x);      \
  EXPECT_DOUBLE_EQ(DATA0.y, DATA1.y);      \
  EXPECT_DOUBLE_EQ(DATA0.z, DATA1.z);      \
  EXPECT_DOUBLE_EQ(DATA0.w, DATA1.w);

#define EXPECT_QUATERNION_NEAR(DATA0, DATA1, TOLERANCE) \
  EXPECT_NEAR(DATA0.x, DATA1.x, TOLERANCE);             \
  EXPECT_NEAR(DATA0.y, DATA1.y, TOLERANCE);             \
  EXPECT_NEAR(DATA0.z, DATA1.z, TOLERANCE);             \
  EXPECT_NEAR(DATA0.w, DATA1.w, TOLERANCE);

#define EXPECT_POSE_EQ(DATA0, DATA1)               \
  EXPECT_POINT_EQ(DATA0.position, DATA1.position); \
  EXPECT_QUATERNION_EQ(DATA0.orientation, DATA1.orientation);

#define EXPECT_POSE_NEAR(DATA0, DATA1, TOLERANCE)               \
  EXPECT_POINT_NEAR(DATA0.position, DATA1.position, TOLERANCE); \
  EXPECT_QUATERNION_NEAR(DATA0.orientation, DATA1.orientation, TOLERANCE);

#define EXPECT_VERTEX_EQ(DATA0, DATA1) \
  EXPECT_FLOAT_EQ(DATA0.x, DATA1.x);   \
  EXPECT_FLOAT_EQ(DATA0.y, DATA1.y);   \
  EXPECT_FLOAT_EQ(DATA0.z, DATA1.z);

#define EXPECT_TRIANGLE_EQ(DATA0, DATA1) \
  EXPECT_EQ(DATA0.v0, DATA1.v0);         \
  EXPECT_EQ(DATA0.v1, DATA1.v1);         \
  EXPECT_EQ(DATA0.v2, DATA1.v2);

#define EXPECT_VERTEX_AND_POINT_EQ(VERTEX, POINT) \
  EXPECT_FLOAT_EQ(VERTEX.x, POINT.x);             \
  EXPECT_FLOAT_EQ(VERTEX.y, POINT.y);             \
  EXPECT_FLOAT_EQ(VERTEX.z, POINT.z);

#define EXPECT_POSITION_NEAR(POSITION0, POSITION1, TOLERANCE) \
  EXPECT_NEAR(POSITION0.x, POSITION1.x, TOLERANCE);           \
  EXPECT_NEAR(POSITION0.y, POSITION1.y, TOLERANCE);           \
  EXPECT_NEAR(POSITION0.z, POSITION1.z, TOLERANCE);

#endif  // SIMPLE_SENSOR_SIMULATOR__TEST__UTILS__EXPECT_EQ_MACROS_HPP_
