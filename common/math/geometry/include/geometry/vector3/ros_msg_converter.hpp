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

#ifndef GEOMETRY__VECTOR3__ROS_MSG_CONVERTER_HPP_
#define GEOMETRY__VECTOR3__ROS_MSG_CONVERTER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace math
{
namespace geometry
{
auto castToVec(const geometry_msgs::msg::Point & p) -> geometry_msgs::msg::Vector3;
auto castToPoint(const geometry_msgs::msg::Vector3 & p) -> geometry_msgs::msg::Point;
}  // namespace geometry
}  // namespace math
#endif  // GEOMETRY__VECTOR3__ROS_MSG_CONVERTER_HPP_
