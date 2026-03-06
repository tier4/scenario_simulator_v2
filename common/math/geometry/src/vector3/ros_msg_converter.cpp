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

#include <geometry/vector3/ros_msg_converter.hpp>

namespace math
{
namespace geometry
{
auto castToVec(const geometry_msgs::msg::Point & p) -> geometry_msgs::msg::Vector3
{
  auto result = geometry_msgs::msg::Vector3();
  result.x = p.x;
  result.y = p.y;
  result.z = p.z;
  return result;
}
auto castToPoint(const geometry_msgs::msg::Vector3 & p) -> geometry_msgs::msg::Point
{
  auto result = geometry_msgs::msg::Point();
  result.x = p.x;
  result.y = p.y;
  result.z = p.z;
  return result;
}
}  // namespace geometry
}  // namespace math
