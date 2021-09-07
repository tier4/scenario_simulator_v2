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

#ifndef TRAFFIC_SIMULATOR__TEST__EXPECT_EQ_MACROS_HPP_
#define TRAFFIC_SIMULATOR__TEST__EXPECT_EQ_MACROS_HPP_

#include <gtest/gtest.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#define EXPECT_POINT_EQ(DATA0, DATA1) \
  EXPECT_DOUBLE_EQ(DATA0.x, DATA1.x); \
  EXPECT_DOUBLE_EQ(DATA0.y, DATA1.y); \
  EXPECT_DOUBLE_EQ(DATA0.z, DATA1.z);

#define EXPECT_VECTOR3_EQ(DATA0, DATA1) \
  EXPECT_DOUBLE_EQ(DATA0.x, DATA1.x);   \
  EXPECT_DOUBLE_EQ(DATA0.y, DATA1.y);   \
  EXPECT_DOUBLE_EQ(DATA0.z, DATA1.z);

#define EXPECT_QUATERNION_EQ(DATA0, DATA1) \
  EXPECT_DOUBLE_EQ(DATA0.x, DATA1.x);      \
  EXPECT_DOUBLE_EQ(DATA0.y, DATA1.y);      \
  EXPECT_DOUBLE_EQ(DATA0.z, DATA1.z);      \
  EXPECT_DOUBLE_EQ(DATA0.w, DATA1.w);

#define EXPECT_POSE_EQ(DATA0, DATA1)               \
  EXPECT_POINT_EQ(DATA0.position, DATA1.position); \
  EXPECT_QUATERNION_EQ(DATA0.orientation, DATA1.orientation);

#define EXPECT_LANELET_POSE_EQ(DATA0, DATA1)     \
  EXPECT_EQ(DATA0.lanelet_id, DATA1.lanelet_id); \
  EXPECT_DOUBLE_EQ(DATA0.s, DATA1.s);            \
  EXPECT_DOUBLE_EQ(DATA0.offset, DATA1.offset);  \
  EXPECT_VECTOR3_EQ(DATA0.rpy, DATA1.rpy);

#endif  // TRAFFIC_SIMULATOR__TEST__EXPECT_EQ_MACROS_HPP_
