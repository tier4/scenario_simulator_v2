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

#ifndef CONTEXT_GAMMA_PLANNER__UTILS_ROS_MSG_CONVERTER_HPP_
#define CONTEXT_GAMMA_PLANNER__UTILS_ROS_MSG_CONVERTER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace context_gamma_planner
{
auto cast_to_vec(const geometry_msgs::msg::Point & p) -> geometry_msgs::msg::Vector3;
auto cast_to_point(const geometry_msgs::msg::Vector3 & p) -> geometry_msgs::msg::Point;
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__UTILS_ROS_MSG_CONVERTER_HPP_
