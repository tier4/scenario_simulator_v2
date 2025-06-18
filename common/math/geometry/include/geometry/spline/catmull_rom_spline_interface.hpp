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
#include <scenario_simulator_exception/exception.hpp>
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
    const std::vector<geometry_msgs::msg::Point> & polygon, const bool search_backward = false,
    const std::optional<std::pair<double, double>> & s_range = std::nullopt) const = 0;
  virtual double getSquaredDistanceIn2D(
    const geometry_msgs::msg::Point & point, const double s) const = 0;

  /**
   * @brief Calculates nearest s value from given range and its squared distance in 2D.
   * This function is guaranteed to work correctly only when the spline is convex or concave - the
   * second derivative of the spline has the same sign over the whole span of the given range
   * (either <= 0 or >= 0).
   */
  auto nearestS(const geometry_msgs::msg::Point & point, double s_start, double s_end) const
    -> std::pair<double, double>
  {
    /**
     * Convergence threshold for binary search.
     * The search stops when the interval between `s_start` and `s_end` is below this value.
     * The value 0.05 was chosen empirically to balance accuracy and performance.
     * A smaller value improves precision but increases computation time.
     */
    constexpr double distance_accuracy{0.05};

    if (s_start < 0.0 or getLength() < s_start or s_end < 0.0 or getLength() < s_end) {
      THROW_SIMULATION_ERROR(
        "Invalid s range: [", s_start, ", ", s_end, "] when spline length is ", getLength());
    }

    /// @note it may be a good idea to develop spline.getSquaredDistanceIn2D(point, s_start, s_end);
    auto s_start_distance = getSquaredDistanceIn2D(point, s_start);
    auto s_end_distance = getSquaredDistanceIn2D(point, s_end);

    while (std::abs(s_start - s_end) > distance_accuracy) {
      double s_mid = (s_start + s_end) / 2.0;
      double s_mid_distance = getSquaredDistanceIn2D(point, s_mid);
      if (s_start_distance > s_end_distance) {
        s_start = s_mid;
        s_start_distance = s_mid_distance;
      } else {
        s_end = s_mid;
        s_end_distance = s_mid_distance;
      }
    }
    return std::make_pair(
      s_start_distance < s_end_distance ? s_start : s_end,
      std::min(s_start_distance, s_end_distance));
  }
};
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__SPLINE__CATMULL_ROM_SPLINE_INTERFACE_HPP_
