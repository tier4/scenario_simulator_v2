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

#include <geometry/spline/catmull_rom_subspline.hpp>
#include <optional>
#include <scenario_simulator_exception/exception.hpp>
#include <vector>

namespace math
{
namespace geometry
{
double CatmullRomSubspline::getLength() const { return end_s_ - start_s_; }

std::optional<double> CatmullRomSubspline::getCollisionPointIn2D(
  const std::vector<geometry_msgs::msg::Point> & polygon, const bool search_backward) const
{
  /// @note Make sure end is greater than start, otherwise the spline is invalid
  if (end_s_ < start_s_) {
    THROW_SIMULATION_ERROR(
      "The start of the subspline is greater than the end. "
      "The start of the subspline should always be less than the end. ",
      "Subspline start: ", start_s_, " Subspline end: ", end_s_, " ",
      "Something completely unexpected happened. ",
      "This message is not originally intended to be displayed, if you see it, please "
      "contact the developer of traffic_simulator.");
  }

  std::set<double> s_value_candidates = spline_->getCollisionPointsIn2D(polygon);

  if (s_value_candidates.empty()) {
    return std::nullopt;
  }

  /// @note Iterators for the range of this subspline
  auto begin = s_value_candidates.lower_bound(start_s_);
  auto end = s_value_candidates.upper_bound(end_s_);

  /**
   * @note If begin == end there is no collision in the given range, or it is past the range
   * If begin == s_value_candidates.end() all elements are less than start_s_
   * end == s_value_candidates.end() is valid, all elements are less than end_s_
   */
  if (begin == end || begin == s_value_candidates.end()) {
    return std::nullopt;
  }

  if (search_backward) {
    return *(--end) - start_s_;  // end is past the last element, so we need the one before
  }
  return *begin - start_s_;
}

auto CatmullRomSubspline::getSquaredDistanceIn2D(
  const geometry_msgs::msg::Point & point, const double s) const -> double
{
  return spline_->getSquaredDistanceIn2D(point, start_s_ + s);
}
}  // namespace geometry
}  // namespace math
