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

#include <map_fragment/constraint.hpp>
#include <map_fragment/filter.hpp>
#include <map_fragment/load_lanelet_map.hpp>
#include <map_fragment/map_fragment.hpp>
#include <map_fragment/selector.hpp>
#include <rclcpp/rclcpp.hpp>

auto main(const int argc, char const * const * const argv) -> int
try {
  using namespace map_fragment;

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node(std::filesystem::path(argv[0]).stem());

  const auto lanelet_map = loadLaneletMap<ParameterSetup::Automatic>(node);

  const auto routing_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map, vehicleTrafficRules());

  auto satisfy_first_lanelet_constraints =
    [&, constraints = loadAllLaneletIDConstraints(node, "first_lanelet_")](const auto & lanelet) {
      return std::all_of(constraints.begin(), constraints.end(), [&](const auto & constraint) {
        return constraint.second(lanelet, *lanelet_map, *routing_graph);
      });
    };

  node.declare_parameter("path_is_allowed_to_include_lane_changes", false);
  node.declare_parameter("path_element_limit", 5);

  auto search_possible_paths = [&](auto && lanelet) {
    auto parameters = lanelet::routing::PossiblePathsParams();
    // clang-format off
    parameters.includeLaneChanges = node.get_parameter("path_is_allowed_to_include_lane_changes").as_bool();
    parameters.includeShorterPaths = true;
    parameters.elementLimit = node.get_parameter("path_element_limit").as_int();
    // clang-format on
    return routing_graph->possiblePaths(lanelet, parameters);
  };

  auto satisfy_path_constraints =
    [&, constraints = loadAllLaneletPathConstraints(node, "path_")](const auto & path) {
      return std::all_of(constraints.begin(), constraints.end(), [&](const auto & constraint) {
        return constraint.second(path, *lanelet_map, *routing_graph);
      });
    };

  auto satisfy_last_lanelet_constraints =
    [&, constraints = loadAllLaneletIDConstraints(node, "last_lanelet_")](const auto & path) {
      return std::all_of(constraints.begin(), constraints.end(), [&](const auto & constraint) {
        return constraint.second(path.back(), *lanelet_map, *routing_graph);
      });
    };

  node.declare_parameter("select", "any");

  auto select = loadLaneletPathSelector(node);

  // clang-format off
  lanelet_map->laneletLayer | curry2(filter)(satisfy_first_lanelet_constraints)
                            | curry2(append_map)(search_possible_paths)
                            | curry2(filter)(satisfy_last_lanelet_constraints)
                            | curry2(filter)(satisfy_path_constraints)
                            | select;
  // clang-format on

  return EXIT_SUCCESS;
} catch (const std::exception & exception) {
  std::cerr << exception.what() << std::endl;
  return EXIT_FAILURE;
}
