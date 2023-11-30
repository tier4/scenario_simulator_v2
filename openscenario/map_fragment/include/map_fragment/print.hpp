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

#ifndef MAP_FRAGMENT__PRINT_HPP_
#define MAP_FRAGMENT__PRINT_HPP_

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/LaneletPath.h>

#include <iostream>

// TODO Move implementations into .cpp file.

namespace map_fragment
{
auto print(std::ostream & ostream, const lanelet::ConstLanelet & lanelet) -> auto &
{
  return ostream << lanelet.id();
}

auto print(std::ostream & ostream, const lanelet::Lanelet & lanelet) -> auto &
{
  return ostream << lanelet.id();
}

auto print(std::ostream & ostream, const lanelet::ConstLanelets & lanelets) -> auto &
{
  for (auto && lanelet : lanelets) {
    print(ostream, lanelet) << std::endl;
  }
  return ostream;
}

auto print(std::ostream & ostream, const lanelet::Lanelets & lanelets) -> auto &
{
  for (auto && lanelet : lanelets) {
    print(ostream, lanelet) << std::endl;
  }
  return ostream;
}

auto print(std::ostream & ostream, const lanelet::routing::LaneletPath & path) -> auto &
{
  ostream << "[";
  for (auto && lanelet : path) {
    print(ostream, lanelet) << (&lanelet != &path.back() ? ", " : "]");
  }
  return ostream;
}

auto print(std::ostream & ostream, const lanelet::routing::LaneletPaths & paths) -> auto &
{
  for (auto && path : paths) {
    print(ostream, path) << std::endl;
  }
  return ostream;
}
}  // namespace map_fragment

#endif  // MAP_FRAGMENT__PRINT_HPP_
