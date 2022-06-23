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

#ifndef TRAFFIC_SIMULATOR__MATH__POLYGON_HPP_
#define TRAFFIC_SIMULATOR__MATH__POLYGON_HPP_

#include <geometry_msgs/msg/point.hpp>

namespace traffic_simulator
{
namespace math
{
std::vector<geometry_msgs::msg::Point> get2DConvexHull(
  const std::vector<geometry_msgs::msg::Point> & points);
}  // namespace math
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__MATH__POLYGON_HPP_
