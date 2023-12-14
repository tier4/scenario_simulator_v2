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
#include <map_fragment/road_segment.hpp>
#include <rclcpp/rclcpp.hpp>

auto main(const int argc, char const * const * const argv) -> int
try {
  using namespace map_fragment;

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node(std::filesystem::path(argv[0]).stem());

  const auto width = [&]() {
    node.declare_parameter("width", default_value::width);
    return node.get_parameter("width").as_double();
  }();

  const auto length = [&]() {
    node.declare_parameter("length", default_value::length);
    return node.get_parameter("length").as_double();
  }();

  const auto curvature = [&]() {
    node.declare_parameter("curvature", default_value::curvature);
    return node.get_parameter("curvature").as_double();
  }();

  const auto number_of_lanes = [&]() {
    node.declare_parameter("number_of_lanes", default_value::number_of_lanes);
    return node.get_parameter("number_of_lanes").as_int();
  }();

  const auto resolution = [&]() {
    node.declare_parameter("resolution", default_value::resolution);
    return node.get_parameter("resolution").as_int();
  }();

  const auto output_directory = [&]() {
    node.declare_parameter("output_directory", default_value::directory());
    return std::filesystem::path(node.get_parameter("output_directory").as_string());
  }();

  const auto segment = makeCurvedRoadSegment(curvature, length, number_of_lanes, width);
  const auto map = lanelet::utils::createMap(segment.getLanelets(resolution));

  map_fragment::write(*map, output_directory);

  std::cout << output_directory.c_str() << std::endl;

  return EXIT_SUCCESS;
} catch (const std::exception & exception) {
  std::cerr << exception.what() << std::endl;
  return EXIT_FAILURE;
}
