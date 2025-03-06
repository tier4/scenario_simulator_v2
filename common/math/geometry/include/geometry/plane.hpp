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

#ifndef GEOMETRY__PLANE_HPP_
#define GEOMETRY__PLANE_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <optional>

namespace math
{
namespace geometry
{

/// @class Plane
/// @brief Represents a plane in 3D space, defined by a normal vector and a point on the plane.
///
/// The plane is described using the equation:
///     Ax + By + Cz + D = 0
/// where:
/// - A, B, C are the components of the normal vector (normal_ attribute).
/// - D is the offset from the origin, calculated using the point and normal vector (d_ attribute).
struct Plane
{
  Plane(const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Vector3 & normal);
  auto offset(const geometry_msgs::msg::Point & point) const -> double;

  const geometry_msgs::msg::Vector3 normal_;
  const double d_;
};
}  // namespace geometry
}  // namespace math
#endif  // GEOMETRY__PLANE_HPP_
