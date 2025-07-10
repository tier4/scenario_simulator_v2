// Copyright 2021 Tier IV, Inc All rights reserved.
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

#ifndef CONTEXT_GAMMA_PLANNER__UTILS_COLLIDER_HPP_
#define CONTEXT_GAMMA_PLANNER__UTILS_COLLIDER_HPP_

#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>

#include "context_gamma_planner/utils/orca.hpp"

namespace context_gamma_planner
{
auto create_box_polygon(double width, double length) -> Polygon;
auto polygon_from_bbox(const traffic_simulator_msgs::msg::BoundingBox & bbox) -> Polygon;
auto rotate_polygon(const Polygon & poly, const geometry_msgs::msg::Quaternion & orientation)
  -> Polygon;
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__UTILS_COLLIDER_HPP_