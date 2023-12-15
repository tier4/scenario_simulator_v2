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

#include <algorithm>
#include <limits>
#include <map_fragment/map_fragment.hpp>

namespace map_fragment
{
inline namespace constraint
{
auto isLeftmost(const lanelet::routing::RoutingGraph & graph, const lanelet::ConstLanelet & lanelet)
{
  return graph.leftRelations(lanelet).empty();
}

auto isRightmost(
  const lanelet::LaneletMap & map, const lanelet::routing::RoutingGraph & graph,
  const lanelet::ConstLanelet & lanelet)
{
  const auto relations = graph.rightRelations(lanelet);

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

auto direction(const lanelet::ConstLineString3d & points)
{
  auto vectors = Eigen::MatrixXd(3, points.size() - 1);

  for (auto iter = points.begin(); iter != std::prev(points.end()); ++iter) {
    vectors.col(std::distance(points.begin(), iter))
      << (std::next(iter)->basicPoint() - iter->basicPoint()).normalized();
  }

  return vectors.rowwise().sum().normalized();
};

template <typename Node>
auto loadLaneletConstraints(const Node & node, const std::string & prefix = "")
{
  std::unordered_map<
    std::string, std::function<bool(
                   const lanelet::ConstLanelet &, const lanelet::LaneletMap &,
                   const lanelet::routing::RoutingGraph &)>>
    constraints;

  if (const auto name = prefix + "type"; node.has_parameter(name)) {
    const auto type = node.get_parameter(name).as_string();
    constraints.emplace(name, [type](auto && lanelet, auto &&...) {  //
      return lanelet.attributeOr("type", "") == type;
    });
  }

  if (const auto name = prefix + "subtype"; node.has_parameter(name)) {
    const auto subtype = node.get_parameter(name).as_string();
    constraints.emplace(name, [subtype](auto && lanelet, auto &&...) {
      return lanelet.attributeOr("subtype", "") == subtype;
    });
  }

  if (const auto name = prefix + "turn_direction"; node.has_parameter(name)) {
    if (const auto turn_direction = node.get_parameter(name).as_string();
        not turn_direction.empty()) {
      constraints.emplace(name, [turn_direction](auto && lanelet, auto &&...) {
        return lanelet.attributeOr("turn_direction", "") == turn_direction;
      });
    }
  }

  if (const auto name = prefix + "id_is_equal_to"; node.has_parameter(name)) {
    if (const auto id = node.get_parameter(name).as_int(); id) {
      constraints.emplace(name, [id](auto && lanelet, auto &&...) {  //
        return lanelet.id() == id;
      });
    }
  }

  if (const auto name = prefix + "id_is_not_equal_to"; node.has_parameter(name)) {
    if (const auto id = node.get_parameter(name).as_int(); id) {
      constraints.emplace(name, [id](auto && lanelet, auto &&...) {  //
        return lanelet.id() != id;
      });
    }
  }

  if (const auto name = prefix + "id_is_greater_than"; node.has_parameter(name)) {
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

  if (const auto name = prefix + "centerline_curvature_less_than"; node.has_parameter(name)) {
    const auto curvature = node.get_parameter(name).as_double();
    constraints.emplace(name, [curvature](auto && lanelet, auto &&...) {
      return std::abs(curvature2d(lanelet.centerline())) < curvature;
    });
  }

  if (const auto name = prefix + "centerline_curvature_greater_than"; node.has_parameter(name)) {
    const auto curvature = node.get_parameter(name).as_double();
    constraints.emplace(name, [curvature](auto && lanelet, auto &&...) {
      return curvature < std::abs(curvature2d(lanelet.centerline()));
    });
  }

  if (const auto name = prefix + "conflicts_with"; node.has_parameter(name)) {
    if (const auto id = node.get_parameter(name).as_int(); id) {
      constraints.emplace(name, [id](auto && lanelet, auto &&, auto && graph) {
        const auto suspects = graph.conflicting(lanelet);
        return std::any_of(
          suspects.begin(), suspects.end(), [&](auto && suspect) { return suspect.id() == id; });
      });
    }
  }

  if (const auto name = prefix + "direction_is_roughly_the_same_as"; node.has_parameter(name)) {
    if (const auto lanelet_id = node.get_parameter(name).as_int(); lanelet_id) {
      constraints.emplace(name, [lanelet_id](auto && lanelet, auto && map, auto &&...) {
        auto found = map.laneletLayer.find(lanelet_id);
        return found != map.laneletLayer.end() and
               0.5 < direction(lanelet.centerline3d()).dot(direction(found->centerline3d()));
      });
    }
  }

  if (const auto name = prefix + "direction_is_roughly_the_opposite_of"; node.has_parameter(name)) {
    if (const auto lanelet_id = node.get_parameter(name).as_int(); lanelet_id) {
      constraints.emplace(name, [lanelet_id](auto && lanelet, auto && map, auto &&...) {
        auto found = map.laneletLayer.find(lanelet_id);
        return found != map.laneletLayer.end() and
               direction(lanelet.centerline3d()).dot(direction(found->centerline3d())) < -0.5;
      });
    }
  }

  if (const auto name = prefix + "related_to_regulatory_element_subtyped";
      node.has_parameter(name)) {
    if (const auto subtype = node.get_parameter(name).as_string(); not subtype.empty()) {
      constraints.emplace(name, [subtype](auto && lanelet, auto &&...) {
        return [&](auto && regulatory_elements) {
          return std::any_of(
            regulatory_elements.begin(), regulatory_elements.end(),
            [&](auto && regulatory_element) {
              return regulatory_element->attributeOr("subtype", "") == subtype;
            });
        }(lanelet.regulatoryElements());
      });
    }
  }

  if (const auto name = prefix + "route_length_greater_than"; node.has_parameter(name)) {
    const auto lower_bound = node.get_parameter(name).as_double();
    constraints.emplace(name, [lower_bound](auto && lanelet, auto &&, auto && graph) {
      return 0 < graph.possiblePaths(lanelet, lower_bound).size();
    });
  }

  return constraints;
}

template <typename Node>
auto loadAllLaneletConstraints(Node & node, const std::string & prefix = "")
{
  if (const auto name = prefix + "lanelet_type"; not node.has_parameter(name)) {
    node.declare_parameter(name, "lanelet");
  }

  if (const auto name = prefix + "subtype"; not node.has_parameter(name)) {
    node.declare_parameter(name, "road");
  }

  if (const auto name = prefix + "turn_direction"; not node.has_parameter(name)) {
    node.declare_parameter(name, "");
  }

  if (const auto name = prefix + "id_is_equal_to"; not node.has_parameter(name)) {
    node.declare_parameter(name, lanelet::Id());
  }

  if (const auto name = prefix + "id_is_not_equal_to"; not node.has_parameter(name)) {
    node.declare_parameter(name, lanelet::Id());
  }

  if (const auto name = prefix + "id_is_greater_than"; not node.has_parameter(name)) {
    node.declare_parameter(name, lanelet::Id());
  }

  if (const auto name = prefix + "id_is_less_than"; not node.has_parameter(name)) {
    node.declare_parameter(name, std::numeric_limits<lanelet::Id>::max());
  }

  if (const auto name = prefix + "is_left_of"; not node.has_parameter(name)) {
    node.declare_parameter(name, lanelet::Id());
  }

  if (const auto name = prefix + "is_right_of"; not node.has_parameter(name)) {
    node.declare_parameter(name, lanelet::Id());
  }

  if (const auto name = prefix + "is_leftmost"; not node.has_parameter(name)) {
    node.declare_parameter(name, false);
  }

  if (const auto name = prefix + "is_rightmost"; not node.has_parameter(name)) {
    node.declare_parameter(name, false);
  }

  if (const auto name = prefix + "is_not_leftmost"; not node.has_parameter(name)) {
    node.declare_parameter(name, false);
  }

  if (const auto name = prefix + "is_not_rightmost"; not node.has_parameter(name)) {
    node.declare_parameter(name, false);
  }

  if (const auto name = prefix + "centerline_curvature_less_than"; not node.has_parameter(name)) {
    node.declare_parameter(name, std::numeric_limits<double>::max());
  }

  if (const auto name = prefix + "centerline_curvature_greater_than";
      not node.has_parameter(name)) {
    node.declare_parameter(name, std::numeric_limits<double>::lowest());
  }

  if (const auto name = prefix + "conflicts_with"; not node.has_parameter(name)) {
    node.declare_parameter(name, lanelet::Id());
  }

  if (const auto name = prefix + "direction_is_roughly_the_same_as"; not node.has_parameter(name)) {
    node.declare_parameter(name, lanelet::Id());
  }

  if (const auto name = prefix + "direction_is_roughly_the_opposite_of"; not node.has_parameter(name)) {
    node.declare_parameter(name, lanelet::Id());
  }

  if (const auto name = prefix + "related_to_regulatory_element_subtyped";
      not node.has_parameter(name)) {
    node.declare_parameter(name, "");
  }

  if (const auto name = prefix + "route_length_greater_than"; not node.has_parameter(name)) {
    node.declare_parameter(name, 0.0);
  }

  return loadLaneletConstraints(node, prefix);
}

template <typename Node>
auto loadLaneletPathConstraints(const Node & node, const std::string & prefix = "")
{
  std::unordered_map<
    std::string, std::function<bool(
                   const lanelet::routing::LaneletPath &, const lanelet::LaneletMap &,
                   const lanelet::routing::RoutingGraph &)>>
    constraints;

  if (const auto name = prefix + "includes_id"; node.has_parameter(name)) {
    if (const auto id = node.get_parameter(name).as_int(); id) {
      constraints.emplace(name, [id](auto && path, auto &&...) {  //
        return std::any_of(
          path.begin(), path.end(), [id](auto && lanelet) { return lanelet.id() == id; });
      });
    }
  }

  if (const auto name = prefix + "is_allowed_to_contain_duplicate_lanelet_ids";
      node.has_parameter(name)) {
    if (not node.get_parameter(name).as_bool()) {
      constraints.emplace(name, [](auto path, auto &&...) {  //
        std::sort(path.begin(), path.end(), [](auto && a, auto && b) { return a.id() < b.id(); });
        return std::adjacent_find(path.begin(), path.end(), [](auto && a, auto && b) {
                 return a.id() == b.id();
               }) == path.end();
      });
    }
  }

  return constraints;
}

template <typename Node>
auto loadAllLaneletPathConstraints(Node & node, const std::string & prefix = "")
{
  if (const auto name = prefix + "includes_id"; not node.has_parameter(name)) {
    node.declare_parameter(name, lanelet::Id());
  }

  if (const auto name = prefix + "is_allowed_to_contain_duplicate_lanelet_ids";
      not node.has_parameter(name)) {
    node.declare_parameter(name, false);
  }

  return loadLaneletPathConstraints(node, prefix);
}
}  // namespace constraint
}  // namespace map_fragment

#endif  // MAP_FRAGMENT__CONSTRAINTS_HPP__
