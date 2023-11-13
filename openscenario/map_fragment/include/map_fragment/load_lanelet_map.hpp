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

#ifndef MAP_FRAGMENT__LOAD_LANELET_MAP_HPP_
#define MAP_FRAGMENT__LOAD_LANELET_MAP_HPP_

#include <lanelet2_core/geometry/LaneletMap.h>

#include <map_fragment/map_fragment.hpp>
#include <rclcpp/rclcpp.hpp>

namespace map_fragment
{
template <typename Node>
auto loadLaneletMap(const Node & node)
{
  const auto input_directory =
    node.has_parameter("input_directory")
      ? std::filesystem::path(node.get_parameter("input_directory").as_string())
      : default_value::directory();

  const auto input_filename = node.has_parameter("input_filename")
                                ? node.get_parameter("input_filename").as_string()
                                : default_value::filename;

  return lanelet::load(input_directory / input_filename, projector());
}
}  // namespace map_fragment

#endif  // MAP_FRAGMENT__LOAD_LANELET_MAP_HPP_
