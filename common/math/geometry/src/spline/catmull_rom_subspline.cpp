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
#include <vector>

namespace math
{
namespace geometry
{
double CatmullRomSubspline::getLength() const { return end_s_ - start_s_; }

std::optional<double> CatmullRomSubspline::getCollisionPointIn2D(
  const std::vector<geometry_msgs::msg::Point> & polygon, const bool search_backward) const
{
  auto s = spline_->getCollisionPointIn2D(polygon, search_backward);

  if (!s) {
    return std::nullopt;
  }

  if (s.value() < start_s_ || end_s_ < s.value()) {
    return std::nullopt;
  }

  return s.value() - start_s_;
}
}  // namespace geometry
}  // namespace math
