/**
 * @file distance.cpp
 * @author Masaya Kataoka (masaya.kataoka@tier4.jp)
 * @brief functions for calculating distance
 * @version 0.1
 * @date 2021-04-01
 *
 * @copyright Copyright(c) TIER IV.Inc {2015}
 *
 */

// Copyright 20152020 TIER IV.inc. All rights reserved.
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

#include <cmath>
#include <traffic_simulator/math/distance.hpp>

namespace traffic_simulator
{
namespace math
{
double getDistance(const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Point & p1)
{
  return std::sqrt(std::pow(p0.x - p1.x, 2) + std::pow(p0.y - p1.y, 2) + std::pow(p0.z - p1.z, 2));
}

double getDistance(const geometry_msgs::msg::Pose & p0, const geometry_msgs::msg::Point & p1)
{
  return getDistance(p0.position, p1);
}

double getDistance(const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Pose & p1)
{
  return getDistance(p0, p1.position);
}

double getDistance(const geometry_msgs::msg::Pose & p0, const geometry_msgs::msg::Pose & p1)
{
  return getDistance(p0.position, p1.position);
}
}  // namespace math
}  // namespace traffic_simulator
