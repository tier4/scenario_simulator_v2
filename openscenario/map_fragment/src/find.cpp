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

  node.declare_parameter("input_directory", default_value::directory());
  node.declare_parameter("input_filename", default_value::filename);

  const auto lanelet_map = loadLaneletMap(node);

  const auto routing_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map, vehicleTrafficRules());

  node.declare_parameter("type", "lanelet");
  node.declare_parameter("subtype", "road");
  node.declare_parameter("id_is_greater_than", lanelet::Id());
  node.declare_parameter("id_is_less_than", std::numeric_limits<lanelet::Id>::max());
  node.declare_parameter("is_left_of", lanelet::Id());
  node.declare_parameter("is_right_of", lanelet::Id());
  node.declare_parameter("is_leftmost", false);
  node.declare_parameter("is_rightmost", false);
  node.declare_parameter("is_not_leftmost", false);
  node.declare_parameter("is_not_rightmost", false);
  node.declare_parameter("route_length_greater_than", 0.0);  // DEPRECATED

  const auto constraints = loadLaneletIDConstraints(node);

  auto satisfy = [&](const auto & lanelet) {
    return std::all_of(constraints.begin(), constraints.end(), [&](const auto & constraint) {
      return constraint.second(lanelet, *lanelet_map, *routing_graph);
    });
  };

  node.declare_parameter("select", "any");

  const auto select = loadLaneletSelector(node);

  filter(satisfy, lanelet_map->laneletLayer, select);

  return EXIT_SUCCESS;
} catch (const std::exception & exception) {
  std::cerr << exception.what() << std::endl;
  return EXIT_FAILURE;
}
