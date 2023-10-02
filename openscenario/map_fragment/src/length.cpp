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

#include <lanelet2_io/Io.h>

#include <lanelet2_extension/utility/utilities.hpp>
#include <map_fragment/map_fragment.hpp>
#include <rclcpp/rclcpp.hpp>

#define DEBUG(...) std::cerr << #__VA_ARGS__ " = " << std::boolalpha << (__VA_ARGS__) << std::endl

auto main(const int argc, char const * const * const argv) -> int
try {
  using namespace map_fragment;

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node(std::filesystem::path(argv[0]).stem());

  const auto input_directory = [&]() {
    node.declare_parameter("input_directory", default_value::directory());
    return std::filesystem::path(node.get_parameter("input_directory").as_string());
  }();

  DEBUG(input_directory);

  const auto lanelet2_map = [&]() {
    node.declare_parameter("lanelet2_map", input_directory / "lanelet2_map.osm");
    return node.get_parameter("lanelet2_map").as_string();
  }();

  DEBUG(lanelet2_map);

  const auto id = [&]() {
    node.declare_parameter("id", 1);
    return node.get_parameter("id").as_int();
  }();

  DEBUG(id);

  const auto map = lanelet::load(lanelet2_map, map_fragment::projector());

  std::cout << lanelet::utils::getLaneletLength3d(map->laneletLayer.get(id)) << std::endl;

  DEBUG(lanelet::utils::getLaneletLength2d(map->laneletLayer.get(id)));
  DEBUG(lanelet::utils::getLaneletLength3d(map->laneletLayer.get(id)));

  return EXIT_SUCCESS;
} catch (const std::exception & exception) {
  std::cerr << exception.what() << std::endl;
  return EXIT_FAILURE;
}
