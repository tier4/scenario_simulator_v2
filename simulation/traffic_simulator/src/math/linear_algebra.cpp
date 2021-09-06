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

#include <traffic_simulator/math/linear_algebra.hpp>

namespace traffic_simulator
{
namespace math
{
double getSize(geometry_msgs::msg::Vector3 vec) { return std::hypot(vec.x, vec.y, vec.z); }

geometry_msgs::msg::Vector3 normalize(geometry_msgs::msg::Vector3 vec)
{
  double size = getSize(vec);
  if (std::fabs(size) <= std::numeric_limits<double>::epsilon()) {
    THROW_SIMULATION_ERROR(
      "size of vector (", vec.x, ",", vec.y, ",", vec.z, ") is, ", size,
      " size of the vector you want to normalize should be over ",
      std::numeric_limits<double>::epsilon());
  }
  vec.x = vec.x / size;
  vec.y = vec.y / size;
  vec.z = vec.z / size;
  return vec;
}

geometry_msgs::msg::Vector3 operator*(geometry_msgs::msg::Vector3 vec, double value)
{
  vec.x = vec.x * value;
  vec.y = vec.y * value;
  vec.z = vec.z * value;
  return vec;
}

geometry_msgs::msg::Vector3 operator*(double value, geometry_msgs::msg::Vector3 vec)
{
  return vec * value;
}
}  // namespace math
}  // namespace traffic_simulator
