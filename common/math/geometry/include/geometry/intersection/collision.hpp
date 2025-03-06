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

#ifndef GEOMETRY__INTERSECTION__COLLISION_HPP_
#define GEOMETRY__INTERSECTION__COLLISION_HPP_

#include <geometry/bounding_box.hpp>
#include <geometry/polygon/polygon.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <vector>

namespace math
{
namespace geometry
{
bool checkCollision2D(
  const geometry_msgs::msg::Pose & pose0, const traffic_simulator_msgs::msg::BoundingBox & bbox0,
  const geometry_msgs::msg::Pose & pose1, const traffic_simulator_msgs::msg::BoundingBox & bbox1);
bool contains(
  const std::vector<geometry_msgs::msg::Point> & polygon, const geometry_msgs::msg::Point & point);
}  // namespace geometry

}  // namespace math

#endif  // GEOMETRY__INTERSECTION__COLLISION_HPP_
