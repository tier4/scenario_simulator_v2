// Copyright 2015 TIER IV.inc. All rights reserved.
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

#ifndef GEOMETRY__SPLINE__CATMULL_ROM_SPLINE_INTERFACE_HPP_
#define GEOMETRY__SPLINE__CATMULL_ROM_SPLINE_INTERFACE_HPP_

#include <exception>
#include <geometry_msgs/msg/point.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace math
{
namespace geometry
{
class CatmullRomSplineInterface
{
public:
  virtual ~CatmullRomSplineInterface() = default;
  virtual double getLength() const = 0;
  virtual std::optional<double> getCollisionPointIn2D(
    const std::vector<geometry_msgs::msg::Point> & polygon,
    const bool search_backward = false) const = 0;
  virtual double getSquaredDistanceIn2D(
    const geometry_msgs::msg::Point & point, const double s) const = 0;
};
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__SPLINE__CATMULL_ROM_SPLINE_INTERFACE_HPP_
