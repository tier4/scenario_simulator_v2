// Copyright 2015-2020 TierIV.inc. All rights reserved.
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

#include <simulation_api/math/catmull_rom_spline.hpp>
#include <vector>

namespace simulation_api
{
namespace math
{
CatmullRomSpline::CatmullRomSpline(std::vector<geometry_msgs::msg::Point> control_points)
{
  size_t n = control_points.size() - 1;
  if (n <= 1) {
    throw SplineInterpolationError("numbers of control points are not enough.");
  }
  for (size_t i = 0; i < n; i++) {
    if (i == 0) {
      double ax = 0;
      double bx = control_points[0].x - 2 * control_points[1].x + control_points[2].x;
      double cx = -3 * control_points[0].x + 4 * control_points[1].x - control_points[2].x;
      double dx = 2 * control_points[0].x;
      double ay = 0;
      double by = control_points[0].y - 2 * control_points[1].y + control_points[2].y;
      double cy = -3 * control_points[0].y + 4 * control_points[1].y - control_points[2].y;
      double dy = 2 * control_points[0].y;
      double az = 0;
      double bz = control_points[0].z - 2 * control_points[1].z + control_points[2].z;
      double cz = -3 * control_points[0].z + 4 * control_points[1].z - control_points[2].z;
      double dz = 2 * control_points[0].z;
      curves_.emplace_back(HermiteCurve(ax, bx, cx, dx, ay, by, cy, dy, az, bz, cz, dz));
    } else if (i == (n - 1)) {
      double ax = 0;
      double bx = control_points[i - 1].x - 2 * control_points[i].x + control_points[i + 1].x;
      double cx = -1 * control_points[i - 1].x + control_points[i + 1].x;
      double dx = 2 * control_points[i].x;
      double ay = 0;
      double by = control_points[i - 1].y - 2 * control_points[i].y + control_points[i + 1].y;
      double cy = -1 * control_points[i - 1].y + control_points[i + 1].y;
      double dy = 2 * control_points[i].y;
      double az = 0;
      double bz = control_points[i - 1].z - 2 * control_points[i].z + control_points[i + 1].z;
      double cz = -1 * control_points[i - 1].z + control_points[i + 1].z;
      double dz = 2 * control_points[i].z;
      curves_.emplace_back(HermiteCurve(ax, bx, cx, dx, ay, by, cy, dy, az, bz, cz, dz));
    } else {
      double ax = -1 * control_points[i - 1].x + 3 * control_points[i].x - 3 *
        control_points[i + 1].x + control_points[i + 2].x;
      double bx = 2 * control_points[i - 1].x - 5 * control_points[i].x + 4 *
        control_points[i + 1].x - control_points[i + 2].x;
      double cx = -control_points[i - 1].x + control_points[i + 1].x;
      double dx = 2 * control_points[i + 1].x;
      double ay = -1 * control_points[i - 1].y + 3 * control_points[i].y - 3 *
        control_points[i + 1].y + control_points[i + 2].y;
      double by = 2 * control_points[i - 1].y - 5 * control_points[i].y + 4 *
        control_points[i + 1].y - control_points[i + 2].y;
      double cy = -control_points[i - 1].y + control_points[i + 1].y;
      double dy = 2 * control_points[i + 1].y;
      double az = -1 * control_points[i - 1].z + 3 * control_points[i].z - 3 *
        control_points[i + 1].z + control_points[i + 2].z;
      double bz = 2 * control_points[i - 1].z - 5 * control_points[i].z + 4 *
        control_points[i + 1].z - control_points[i + 2].z;
      double cz = -control_points[i - 1].z + control_points[i + 1].z;
      double dz = 2 * control_points[i + 1].z;
      curves_.emplace_back(HermiteCurve(ax, bx, cx, dx, ay, by, cy, dy, az, bz, cz, dz));
    }
  }
}
}  // namespace math
}  // namespace simulation_api
