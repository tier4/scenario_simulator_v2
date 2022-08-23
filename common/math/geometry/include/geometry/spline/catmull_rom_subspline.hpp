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

#ifndef GEOMETRY__SPLINE__CATMULL_ROM_SUBSPLINE_HPP_
#define GEOMETRY__SPLINE__CATMULL_ROM_SUBSPLINE_HPP_

#include <exception>
#include <geometry/spline/catmull_rom_spline.hpp>
#include <geometry/spline/catmull_rom_spline_interface.hpp>
#include <geometry/spline/hermite_curve.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <string>
#include <utility>
#include <vector>

namespace math
{
namespace geometry
{
class CatmullRomSubspline : public CatmullRomSplineInterface
{
public:
  explicit CatmullRomSubspline(
    std::shared_ptr<math::geometry::CatmullRomSpline> spline, double start_s, double end_s)
  : spline_(spline), start_s_(start_s), end_s_(end_s)
  {
  }

  double getLength() const override;

  boost::optional<double> getCollisionPointIn2D(
    const std::vector<geometry_msgs::msg::Point> & polygon, bool search_backward = false,
    bool close_start_end = true) const override;

private:
  std::shared_ptr<math::geometry::CatmullRomSpline> spline_;
  double start_s_;
  double end_s_;
};
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__SPLINE__CATMULL_ROM_SUBSPLINE_HPP_
