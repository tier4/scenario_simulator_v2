// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include <gtest/gtest.h>

#include <simulation_api/math/hermite_curve.hpp>

TEST(math, HermiteCurve1)
{
  geometry_msgs::msg::Pose start_pose, goal_pose;
  geometry_msgs::msg::Vector3 start_vec, goal_vec;
  goal_pose.position.x = 1;
  start_vec.x = 1;
  goal_vec.x = 1;
  simulation_api::math::HermiteCurve curve
    (start_pose, goal_pose, start_vec, goal_vec);
  EXPECT_DOUBLE_EQ(curve.getLength(), 1);
  EXPECT_DOUBLE_EQ(curve.getPoint(0.5, false).x, 0.5);
  EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).x, 1);
  EXPECT_DOUBLE_EQ(curve.getMaximu2DCurvature(), 0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
