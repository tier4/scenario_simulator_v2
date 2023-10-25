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
#include <map_fragment/map_fragment.hpp>
#include <map_fragment/printer.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>

auto main(const int argc, char const * const * const argv) -> int
try {
  using namespace map_fragment;

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node(std::filesystem::path(argv[0]).stem());

  const auto input_directory = [&]() {
    node.declare_parameter("input_directory", default_value::directory());
    return std::filesystem::path(node.get_parameter("input_directory").as_string());
  }();

  const auto lanelet2_map = [&]() {
    node.declare_parameter("lanelet2_map", input_directory / "lanelet2_map.osm");
    return node.get_parameter("lanelet2_map").as_string();
  }();

  const auto map = lanelet::load(lanelet2_map, map_fragment::projector());

  const auto routing_graph = lanelet::routing::RoutingGraph::build(*map, vehicleTrafficRules());

  node.declare_parameter("type", "lanelet");
  node.declare_parameter("subtype", "road");
  node.declare_parameter("id_greater_than", lanelet::Id());
  node.declare_parameter("id_less_than", std::numeric_limits<lanelet::Id>::max());
  node.declare_parameter("left_of", lanelet::Id());
  node.declare_parameter("right_of", lanelet::Id());
  node.declare_parameter("leftmost", false);
  node.declare_parameter("not_leftmost", false);
  node.declare_parameter("rightmost", false);
  node.declare_parameter("not_rightmost", false);
  node.declare_parameter("route_length_greater_than", 0.0);

  const auto constraints = loadBasicConstraints(node);

  auto satisfy = [&](const auto & lanelet) {
    return std::all_of(constraints.begin(), constraints.end(), [&](const auto & constraint) {
      return constraint.second(lanelet, *map, *routing_graph);
    });
  };

  node.declare_parameter("select", "any");

  filter(satisfy, map->laneletLayer, loadBasicPrinter(node));

  return EXIT_SUCCESS;
} catch (const std::exception & exception) {
  std::cerr << exception.what() << std::endl;
  return EXIT_FAILURE;
}
