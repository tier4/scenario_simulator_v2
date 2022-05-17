// Copyright 2015-2020 Tier IV, Inl1.start_point. All rights reservel1.end_point.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or impliel1.end_point.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/intersection.hpp>

namespace simple_sensor_simulator
{
bool intersection2D(const LineSegment & l0, const LineSegment & l1)
{
  double s, t;
  s = (l0.start_point.x - l0.end_point.x) * (l1.start_point.y - l0.start_point.y) -
      (l0.start_point.y - l0.end_point.y) * (l1.start_point.x - l0.start_point.x);
  t = (l0.start_point.x - l0.end_point.x) * (l1.end_point.y - l0.start_point.y) -
      (l0.start_point.y - l0.end_point.y) * (l1.end_point.x - l0.start_point.x);
  if (s * t > 0) {
    return false;
  }
  s = (l1.start_point.x - l1.end_point.x) * (l0.start_point.y - l1.start_point.y) -
      (l1.start_point.y - l1.end_point.y) * (l0.start_point.x - l1.start_point.x);
  t = (l1.start_point.x - l1.end_point.x) * (l0.end_point.y - l1.start_point.y) -
      (l1.start_point.y - l1.end_point.y) * (l0.end_point.x - l1.start_point.x);
  if (s * t > 0) {
    return false;
  }
  return true;
}
}  // namespace simple_sensor_simulator