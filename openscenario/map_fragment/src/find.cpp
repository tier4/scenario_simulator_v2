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

#include <map_fragment/map_fragment.hpp>
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

  const auto type = [&]() {
    node.declare_parameter("type", "lanelet");
    return node.get_parameter("type").as_string();
  }();

  const auto subtype = [&]() {
    node.declare_parameter("subtype", "road");
    return node.get_parameter("subtype").as_string();
  }();

  auto satisfies_lower_bound_id = [&]() {
    node.declare_parameter("lane_id_greater_than", lanelet::Id());
    const auto lower_bound = node.get_parameter("lane_id_greater_than").as_int();
    return [lower_bound](const auto & lanelet) { return lower_bound < lanelet.id(); };
  }();

  auto satisfies_upper_bound_id = [&]() {
    node.declare_parameter("lane_id_less_than", std::numeric_limits<lanelet::Id>::max());
    const auto upper_bound = node.get_parameter("lane_id_less_than").as_int();
    return [upper_bound](const auto & lanelet) { return lanelet.id() < upper_bound; };
  }();

  auto routing_graph = lanelet::routing::RoutingGraph::build(*map, vehicleTrafficRules());

  auto satisfies_lower_bound_route_length = [&]() {
    node.declare_parameter("route_length_greater_than", 0.0);
    const auto lower_bound = node.get_parameter("route_length_greater_than").as_double();
    return [&, lower_bound](const auto & lanelet) {
      return 0 < routing_graph->possiblePaths(lanelet, lower_bound).size();
    };
  }();

  auto satisfy = [&](const auto & lanelet) {
    auto matches = [&](const auto & attribute, const auto & value) {
      auto iterator = lanelet.attributes().find(attribute);
      return iterator != lanelet.attributes().end() and iterator->second == value;
    };

    return matches("type", type) and              //
           matches("subtype", subtype) and        //
           satisfies_lower_bound_id(lanelet) and  //
           satisfies_upper_bound_id(lanelet) and  //
           satisfies_lower_bound_route_length(lanelet);
  };

  const auto count = std::count_if(map->laneletLayer.begin(), map->laneletLayer.end(), satisfy);

  DEBUG(count);

  if (auto iterator = std::find_if(map->laneletLayer.begin(), map->laneletLayer.end(), satisfy);
      iterator != map->laneletLayer.end()) {
    std::cout << iterator->id() << std::endl;
    return EXIT_SUCCESS;
  } else {
    throw std::runtime_error("not found");
  }
} catch (const std::exception & exception) {
  std::cerr << exception.what() << std::endl;
  return EXIT_FAILURE;
}
