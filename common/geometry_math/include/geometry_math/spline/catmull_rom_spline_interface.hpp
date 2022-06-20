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

#ifndef GEOMETRY_MATH__SPLINE__CATMULL_ROM_SPLINE_INTERFACE_HPP_
#define GEOMETRY_MATH__SPLINE__CATMULL_ROM_SPLINE_INTERFACE_HPP_

#include <boost/optional.hpp>
#include <exception>
#include <geometry_msgs/msg/point.hpp>
#include <string>
#include <utility>
#include <vector>

namespace geometry_math
{
class CatmullRomSplineInterface
{
public:
  virtual double getLength() const = 0;
  virtual boost::optional<double> getCollisionPointIn2D(
    const std::vector<geometry_msgs::msg::Point> & polygon, bool search_backward = false,
    bool close_start_end = true) const = 0;
};
}  // namespace geometry_math

#endif  // GEOMETRY_MATH__SPLINE__CATMULL_ROM_SPLINE_INTERFACE_HPP_
