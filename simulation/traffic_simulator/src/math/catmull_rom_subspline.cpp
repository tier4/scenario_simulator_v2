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

#include <boost/optional.hpp>
#include <traffic_simulator/math/catmull_rom_subspline.hpp>
#include <vector>

namespace traffic_simulator
{
namespace math
{
double CatmullRomSubspline::getLength() const { return end_s_ - start_s_; }

boost::optional<double> CatmullRomSubspline::getCollisionPointIn2D(
  const std::vector<geometry_msgs::msg::Point> & polygon, bool search_backward,
  bool close_start_end) const
{
  auto s = spline_->getCollisionPointIn2D(polygon, search_backward, close_start_end);

  if (!s) {
    return boost::none;
  }

  if (s.get() < start_s_ || end_s_ < s.get()) {
    return boost::none;
  }

  return s.get() - start_s_;
}

}  // namespace math
}  // namespace traffic_simulator
