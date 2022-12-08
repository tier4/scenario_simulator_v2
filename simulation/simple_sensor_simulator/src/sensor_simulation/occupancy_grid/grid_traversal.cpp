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

#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/grid_traversal.hpp>

namespace simple_sensor_simulator
{
// A Fast Voxel Traversal Algorithm for Ray Tracing
// https://www.researchgate.net/publication/2611491_A_Fast_Voxel_Traversal_Algorithm_for_Ray_Tracing

GridTraversal::GridTraversal(double start_x, double start_y, double end_x, double end_y)
: start_x_(start_x),
  start_y_(start_y),
  vx_(end_x - start_x),
  vy_(end_y - start_y),
  step_x_(std::copysign(1.0, vx_)),
  step_y_(std::copysign(1.0, vy_)),
  tdx_(step_x_ / vx_),
  tdy_(step_y_ / vy_)
{
}

auto GridTraversal::begin() const -> Iterator
{
  double tx = vx_ > 0 ? std::ceil(start_x_) : std::floor(start_x_);
  double ty = vy_ > 0 ? std::ceil(start_y_) : std::floor(start_y_);
  tx = vx_ != 0 ? (tx - start_x_) / vx_ : tdx_;
  ty = vy_ != 0 ? (ty - start_y_) / vy_ : tdy_;
  return {this, tx, ty, int32_t(start_x_), int32_t(start_y_)};
}

auto GridTraversal::end() const -> Sentinel { return {}; }

GridTraversal::Iterator::Iterator(
  const GridTraversal * parent, double tx, double ty, int32_t x, int32_t y)
: parent_(parent), tx_(tx), ty_(ty), x_(x), y_(y)
{
}

auto GridTraversal::Iterator::operator*() -> Index { return {x_, y_}; }

auto GridTraversal::Iterator::operator++() -> Iterator &
{
  if (tx_ < ty_) {
    tx_ += parent_->tdx_;
    x_ += parent_->step_x_;
  } else {
    ty_ += parent_->tdy_;
    y_ += parent_->step_y_;
  }
  return *this;
}

auto GridTraversal::Iterator::operator!=(const Sentinel &) -> bool
{
  return tx_ <= 1.0 || ty_ <= 1.0;
}

}  // namespace simple_sensor_simulator
