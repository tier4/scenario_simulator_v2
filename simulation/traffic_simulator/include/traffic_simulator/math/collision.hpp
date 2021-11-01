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

#ifndef TRAFFIC_SIMULATOR__MATH__COLLISION_HPP_
#define TRAFFIC_SIMULATOR__MATH__COLLISION_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <traffic_simulator/math/bounding_box.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <vector>

namespace traffic_simulator
{
namespace math
{
bool checkCollision2D(
  geometry_msgs::msg::Pose pose0, traffic_simulator_msgs::msg::BoundingBox bbox0,
  geometry_msgs::msg::Pose pose1, traffic_simulator_msgs::msg::BoundingBox bbox1);
}  // namespace math
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__MATH__COLLISION_HPP_
