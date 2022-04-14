// Copyright 2015-2022 TierIV.inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__MATH__CATMULL_ROM_SUBSPLINE_HPP_
#define TRAFFIC_SIMULATOR__MATH__CATMULL_ROM_SUBSPLINE_HPP_

#include <exception>
#include <geometry_msgs/msg/point.hpp>
#include <string>
#include <traffic_simulator/math/catmull_rom_interface.hpp>
#include <traffic_simulator/math/catmull_rom_spline.hpp>
#include <traffic_simulator/math/hermite_curve.hpp>
#include <utility>
#include <vector>

namespace traffic_simulator
{
namespace math
{
class CatmullRomSubspline : public CatmullRomInterface
{
public:
  explicit CatmullRomSubspline(
    std::shared_ptr<traffic_simulator::math::CatmullRomSpline> spline, double start_s, double end_s)
  : spline_(spline), start_s_(start_s), end_s_(end_s)
  {
  }

  double getLength() const override;

  boost::optional<double> getCollisionPointIn2D(
    const std::vector<geometry_msgs::msg::Point> & polygon, bool search_backward = false,
    bool close_start_end = true) const override;

private:
  std::shared_ptr<traffic_simulator::math::CatmullRomSpline> spline_;
  double start_s_;
  double end_s_;
};
}  // namespace math
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__MATH__CATMULL_ROM_SUBSPLINE_HPP_
