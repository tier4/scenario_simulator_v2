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

#ifndef MAP_FRAGMENT__LESS_HPP_
#define MAP_FRAGMENT__LESS_HPP_

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/LaneletPath.h>

#include <algorithm>

// TODO Move implementations into .cpp file.

namespace map_fragment
{
auto operator<(const lanelet::ConstLanelet & a, const lanelet::ConstLanelet & b)
{
  return a.id() < b.id();
}

auto operator<(const lanelet::routing::LaneletPath & a, const lanelet::routing::LaneletPath & b)
{
  return std::lexicographical_compare(
    a.begin(), a.end(), b.begin(), b.end(), [](auto && a, auto && b) { return a < b; });
}
}  // namespace map_fragment

#endif  // MAP_FRAGMENT__LESS_HPP_
