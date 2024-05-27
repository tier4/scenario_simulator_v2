// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__UTILS__LANELET_POSE_HPP_
#define TRAFFIC_SIMULATOR__UTILS__LANELET_POSE_HPP_

#include <traffic_simulator/utils/lanelet/lanelet_map.hpp>

namespace traffic_simulator
{
inline namespace lanelet_map
{
template <typename... Ts>
auto activate(Ts &&... xs)
{
  return lanelet2::LaneletMap::activate(std::forward<decltype(xs)>(xs)...);
}

}  // namespace pose
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_POSE_HPP_
