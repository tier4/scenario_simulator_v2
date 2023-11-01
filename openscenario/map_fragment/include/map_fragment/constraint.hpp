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

#ifndef MAP_FRAGMENT__CONSTRAINTS_HPP__
#define MAP_FRAGMENT__CONSTRAINTS_HPP__

#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

namespace map_fragment
{
inline namespace constraint
{
auto isLeftmost(const lanelet::routing::RoutingGraph & graph, const lanelet::Lanelet & lanelet)
{
  return graph.leftRelations(lanelet).empty();
}

auto isRightmost(
  const lanelet::LaneletMap & map, const lanelet::routing::RoutingGraph & graph,
  const lanelet::Lanelet & lanelet)
{
  auto relations = graph.rightRelations(lanelet);

  return relations.empty() or
         std::any_of(relations.begin(), relations.end(), [&](const auto & relation) {
           /*
              NOTE: The Lanelet returned by rightRelations is not useful
              because the directions are aligned regardless of the original
              orientation.
           */
           return lanelet.rightBound() ==
                  map.laneletLayer.find(relation.lanelet.id())->rightBound().invert();
         });
}

template <typename Node>
auto loadLaneletIDConstraints(const Node & node, const std::string & prefix = "")
{
  std::unordered_map<
    std::string, std::function<bool(
                   const lanelet::Lanelet &, const lanelet::LaneletMap &,
                   const lanelet::routing::RoutingGraph &)>>
    constraints;

  if (const auto name = prefix + "type"; node.has_parameter(name)) {
    const auto type = node.get_parameter(name).as_string();
    constraints.emplace(name, [type](auto && lanelet, auto &&...) {  //
      return lanelet.attribute("type") == type;
    });
  }

  if (const auto name = prefix + "subtype"; node.has_parameter(name)) {
    const auto subtype = node.get_parameter(name).as_string();
    constraints.emplace(name, [subtype](auto && lanelet, auto &&...) {
      return lanelet.attribute("subtype") == subtype;
    });
  }

  if (const auto name = prefix + "id_greater_than"; node.has_parameter(name)) {
    const auto lower_bound = node.get_parameter(name).as_int();
    constraints.emplace(name, [lower_bound](auto && lanelet, auto &&...) {  //
      return lower_bound < lanelet.id();
    });
  }

  if (const auto name = prefix + "id_is_less_than"; node.has_parameter(name)) {
    const auto upper_bound = node.get_parameter(name).as_int();
    constraints.emplace(name, [upper_bound](auto && lanelet, auto &&...) {  //
      return lanelet.id() < upper_bound;
    });
  }

  if (const auto name = prefix + "is_left_of"; node.has_parameter(name)) {
    if (const auto right_id = node.get_parameter(name).as_int(); right_id) {
      constraints.emplace(name, [right_id](auto && lanelet, auto && map, auto &&) {
        auto is_left_of = [&](auto right_id) {
          auto right = map.laneletLayer.find(right_id);
          return right != map.laneletLayer.end() and lanelet::geometry::leftOf(lanelet, *right);
        };
        return is_left_of(right_id);
      });
    }
  }

  if (const auto name = prefix + "is_right_of"; node.has_parameter(name)) {
    if (const auto left_id = node.get_parameter(name).as_int(); left_id) {
      constraints.emplace(name, [left_id](auto && lanelet, auto && map, auto &&) {
        auto is_right_of = [&](auto left_id) {
          auto left = map.laneletLayer.find(left_id);
          return left != map.laneletLayer.end() and lanelet::geometry::rightOf(lanelet, *left);
        };
        return is_right_of(left_id);
      });
    }
  }

  if (const auto name = prefix + "is_leftmost";
      node.has_parameter(name) and node.get_parameter(name).as_bool()) {
    constraints.emplace(name, [](auto && lanelet, auto &&, auto && graph) {  //
      return isLeftmost(graph, lanelet);
    });
  }

  if (const auto name = prefix + "is_rightmost";
      node.has_parameter(name) and node.get_parameter(name).as_bool()) {
    constraints.emplace(name, [](auto && lanelet, auto && map, auto && graph) {
      return isRightmost(map, graph, lanelet);
    });
  }

  if (const auto name = prefix + "is_not_leftmost";
      node.has_parameter(name) and node.get_parameter(name).as_bool()) {
    constraints.emplace(name, [](auto && lanelet, auto &&, auto && graph) {  //
      return not isLeftmost(graph, lanelet);
    });
  }

  if (const auto name = prefix + "is_not_rightmost";
      node.has_parameter(name) and node.get_parameter(name).as_bool()) {
    constraints.emplace(name, [](auto && lanelet, auto && map, auto && graph) {
      return not isRightmost(map, graph, lanelet);
    });
  }

  if (const auto name = prefix + "route_length_greater_than"; node.has_parameter(name)) {
    const auto lower_bound = node.get_parameter(name).as_double();
    constraints.emplace(name, [lower_bound](auto && lanelet, auto &&, auto && graph) {
      return 0 < graph.possiblePaths(lanelet, lower_bound).size();
    });
  }

  return constraints;
}
}  // namespace constraint
}  // namespace map_fragment

#endif  // MAP_FRAGMENT__CONSTRAINTS_HPP__
