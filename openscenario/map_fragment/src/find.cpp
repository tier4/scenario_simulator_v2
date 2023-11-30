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

  const auto constraints = loadAllLaneletIDConstraints(node);

  auto satisfy = [&](const auto & lanelet) {
    return std::all_of(constraints.begin(), constraints.end(), [&](const auto & constraint) {
      return constraint.second(lanelet, *lanelet_map, *routing_graph);
    });
  };

  node.declare_parameter("select", "any");

  const auto select = loadBasicSelector(node);

  filter(satisfy, lanelet_map->laneletLayer, select);

  return EXIT_SUCCESS;
} catch (const std::exception & exception) {
  std::cerr << exception.what() << std::endl;
  return EXIT_FAILURE;
}
