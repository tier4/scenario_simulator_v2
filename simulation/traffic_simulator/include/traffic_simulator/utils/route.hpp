// Copyright 2015 Tier IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__UTILS__ROUTE_HPP_
#define TRAFFIC_SIMULATOR__UTILS__ROUTE_HPP_

#include <geometry/spline/catmull_rom_spline.hpp>
#include <geometry/spline/catmull_rom_spline_interface.hpp>
#include <geometry/spline/hermite_curve.hpp>
#include <traffic_simulator/data_type/lane_change.hpp>
#include <traffic_simulator/data_type/lanelet_pose.hpp>
#include <traffic_simulator/lanelet_wrapper/route.hpp>

namespace traffic_simulator
{
inline namespace route
{
template <typename... Ts>
auto route(Ts &&... xs)
{
  return lanelet_wrapper::route::route(std::forward<decltype(xs)>(xs)...);
}
}  // namespace route
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__ROUTE_HPP_
