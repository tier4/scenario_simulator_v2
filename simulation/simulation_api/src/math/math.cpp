// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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
#include <simulation_api/math/hermite_curve.hpp>

int main()
{
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1;
  p1.x = 1;
  p1.y = 3;
  geometry_msgs::msg::Point p2;
  p2.x = 2;
  p2.y = 5;
  geometry_msgs::msg::Point p3;
  p3.x = 4;
  p3.y = 6;
  auto points = {p0, p1, p2, p3};
  auto spline = simulation_api::math::CatmullRomSpline(points);
}
