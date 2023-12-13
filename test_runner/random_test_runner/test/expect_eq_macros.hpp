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
//
// Co-developed by TIER IV, Inc. and Robotec.AI sp. z o.o.

#ifndef RANDOM_TEST_RUNNER__TEST__EXPECT_EQ_MACROS_HPP_
#define RANDOM_TEST_RUNNER__TEST__EXPECT_EQ_MACROS_HPP_

#include <gtest/gtest.h>

#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

#define EXPECT_LANELET_POSE_NEAR(DATA0, DATA1, TOLERANCE) \
  EXPECT_EQ(DATA0.lanelet_id, DATA1.lanelet_id);          \
  EXPECT_NEAR(DATA0.s, DATA1.s, TOLERANCE);

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

#define EXPECT_POSE_NEAR(DATA0, DATA1, TOLERANCE)               \
  EXPECT_POINT_NEAR(DATA0.position, DATA1.position, TOLERANCE); \
  EXPECT_QUATERNION_NEAR(DATA0.orientation, DATA1.orientation, TOLERANCE);

#define EXPECT_IN_RANGE(VAL, MIN, MAX) \
  EXPECT_GE((VAL), (MIN));             \
  EXPECT_LE((VAL), (MAX))

#define EXPECT_NPC_DESCRIPTION_NEAR(DATA0, DATA1, TOLERANCE) \
  EXPECT_STREQ(DATA0.name.c_str(), DATA1.name.c_str());      \
  EXPECT_NEAR(DATA0.speed, DATA1.speed, TOLERANCE);          \
  EXPECT_LANELET_POSE_NEAR(DATA0.start_position, DATA1.start_position, TOLERANCE);

#endif  // RANDOM_TEST_RUNNER__TEST__EXPECT_EQ_MACROS_HPP_
