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

#define EXPECT_POINT_EQ(P0, P1) \
  EXPECT_DOUBLE_EQ(P0.x, P1.x); \
  EXPECT_DOUBLE_EQ(P0.y, P1.y); \
  EXPECT_DOUBLE_EQ(P0.z, P1.z);

#define EXPECT_VECTOR3_EQ(V0, V1) \
  EXPECT_DOUBLE_EQ(V0.x, V1.x);   \
  EXPECT_DOUBLE_EQ(V0.y, V1.y);   \
  EXPECT_DOUBLE_EQ(V0.z, V1.z);

#endif  // TRAFFIC_SIMULATOR__TEST__EXPECT_EQ_MACROS_HPP_
