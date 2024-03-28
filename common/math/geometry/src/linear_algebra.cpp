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

#include <geometry/linear_algebra.hpp>
#include <scenario_simulator_exception/exception.hpp>

namespace math
{
namespace geometry
{
double innerProduct(const geometry_msgs::msg::Vector3 & v0, const geometry_msgs::msg::Vector3 & v1)
{
  return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z;
}

double getInternalAngle(
  const geometry_msgs::msg::Vector3 & v0, const geometry_msgs::msg::Vector3 & v1)
{
  const auto val = innerProduct(v0, v1) / (getSize(v0) * getSize(v1));
  if (-1 <= val && val <= 1) {
    return std::acos(val);
  }
  THROW_SIMULATION_ERROR(
    "value of v0*v1/(size(v0)*size(v1)) in vector v0 : ", v0.x, ",", v0.y, ",", v0.z,
    " and v1 : ", v1.x, ",", v1.y, ",", v1.z, "is out of range, value = ", val);
}

geometry_msgs::msg::Vector3 vector3(double x, double y, double z)
{
  geometry_msgs::msg::Vector3 vec;
  vec.x = x;
  vec.y = y;
  vec.z = z;
  return vec;
}

double getSize(geometry_msgs::msg::Vector3 vec)
{
  return std::sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

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
}  // namespace geometry
}  // namespace math

geometry_msgs::msg::Vector3 operator/(const geometry_msgs::msg::Vector3 & vec, double value)
{
  geometry_msgs::msg::Vector3 ret;
  ret.x = vec.x / value;
  ret.y = vec.y / value;
  ret.z = vec.z / value;
  return ret;
}

geometry_msgs::msg::Vector3 operator*(const geometry_msgs::msg::Vector3 & vec, double value)
{
  geometry_msgs::msg::Vector3 ret;
  ret.x = vec.x * value;
  ret.y = vec.y * value;
  ret.z = vec.z * value;
  return ret;
}

geometry_msgs::msg::Vector3 operator*(double value, const geometry_msgs::msg::Vector3 & vec)
{
  return vec * value;
}

geometry_msgs::msg::Point operator*(const geometry_msgs::msg::Point & point, const double value)
{
  return geometry_msgs::build<geometry_msgs::msg::Point>()
    .x(point.x * value)
    .y(point.z * value)
    .z(point.z * value);
}

geometry_msgs::msg::Point operator*(const double value, const geometry_msgs::msg::Point & point)
{
  return point * value;
}

geometry_msgs::msg::Point operator+(
  const geometry_msgs::msg::Point & v0, const geometry_msgs::msg::Vector3 & v1)
{
  geometry_msgs::msg::Point ret;
  ret.x = v0.x + v1.x;
  ret.y = v0.y + v1.y;
  ret.z = v0.z + v1.z;
  return ret;
}

geometry_msgs::msg::Vector3 operator+(
  const geometry_msgs::msg::Vector3 & v0, const geometry_msgs::msg::Vector3 & v1)
{
  geometry_msgs::msg::Vector3 ret;
  ret.x = v0.x + v1.x;
  ret.y = v0.y + v1.y;
  ret.z = v0.z + v1.z;
  return ret;
}

geometry_msgs::msg::Point operator+(
  const geometry_msgs::msg::Point & v0, const geometry_msgs::msg::Point & v1)
{
  geometry_msgs::msg::Point ret;
  ret.x = v0.x + v1.x;
  ret.y = v0.y + v1.y;
  ret.z = v0.z + v1.z;
  return ret;
}

geometry_msgs::msg::Point operator-(
  const geometry_msgs::msg::Point & v0, const geometry_msgs::msg::Vector3 & v1)
{
  geometry_msgs::msg::Point ret;
  ret.x = v0.x - v1.x;
  ret.y = v0.y - v1.y;
  ret.z = v0.z - v1.z;
  return ret;
}

geometry_msgs::msg::Vector3 operator-(
  const geometry_msgs::msg::Vector3 & v0, const geometry_msgs::msg::Vector3 & v1)
{
  geometry_msgs::msg::Vector3 ret;
  ret.x = v0.x - v1.x;
  ret.y = v0.y - v1.y;
  ret.z = v0.z - v1.z;
  return ret;
}

geometry_msgs::msg::Point operator-(
  const geometry_msgs::msg::Point & v0, const geometry_msgs::msg::Point & v1)
{
  geometry_msgs::msg::Point ret;
  ret.x = v0.x - v1.x;
  ret.y = v0.y - v1.y;
  ret.z = v0.z - v1.z;
  return ret;
}
