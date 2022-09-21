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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_TRAVERSAL_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_TRAVERSAL_HPP_

#include <cmath>
#include <cstdint>
#include <utility>

namespace simple_sensor_simulator
{
class GridTraversal
{
  struct Index
  {
    int32_t x, y;
  };
  struct Sentinel
  {
  };

  class Iterator
  {
  public:
    Iterator(const GridTraversal * parent, double tx, double ty, int32_t x, int32_t y);

    auto operator*() -> Index;
    auto operator++() -> Iterator &;
    auto operator!=(const Sentinel &) -> bool;

  private:
    const GridTraversal * parent_;

    double tx_, ty_;
    int32_t x_, y_;
  };

public:
  GridTraversal(double start_x, double start_y, double end_x, double end_y);

  auto begin() const -> Iterator;
  auto end() const -> Sentinel;

private:
  double start_x_, start_y_;
  double vx_, vy_;

  int32_t step_x_, step_y_;
  double tdx_, tdy_;
};

}  // namespace simple_sensor_simulator

#endif
