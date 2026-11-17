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

#ifndef OPENSCENARIO_INTERPRETER__CMATH__HYPOT_HPP_
#define OPENSCENARIO_INTERPRETER__CMATH__HYPOT_HPP_

#include <cmath>
#include <get_parameter/get_parameter.hpp>

namespace openscenario_interpreter
{
inline namespace cmath
{
/*
   @todo: after checking all the scenario work well with
   consider_pose_by_road_slope = true, remove this function and use
   std::hypot(x, y, z)
*/
template <typename T>
auto hypot(T x, T y, T z)
{
  static const auto consider_pose_by_road_slope =
    common::getParameter<bool>("consider_pose_by_road_slope", false);

  return consider_pose_by_road_slope ? std::hypot(x, y, z) : std::hypot(x, y);
}
}  // namespace cmath
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__CMATH__HYPOT_HPP_
