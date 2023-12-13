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

  const auto before_length = [&]() {
    node.declare_parameter("before_length", default_value::length);
    return node.get_parameter("before_length").as_double();
  }();

  const auto after_length = [&]() {
    node.declare_parameter("after_length", default_value::length);
    return node.get_parameter("after_length").as_double();
  }();

  const auto angle_in_degrees = [&]() {
    node.declare_parameter("angle", default_value::angle);
    return node.get_parameter("angle").as_double();
  }();

  const auto number_of_lanes_per_direction = [&]() {
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

  const auto angle_in_radians = angle_in_degrees * M_PI / 180;

  const std::vector<ParametricCurve::ConstSharedPointer> guide_curve_segments = {

    // Straight segment before the turn
    std::make_shared<Straight>(before_length),

    // Turn segment (or straight if angle is zero)
    angle_in_radians == 0 ? std::static_pointer_cast<ParametricCurve>(  //
                              std::make_shared<Straight>(length))
                          : std::static_pointer_cast<ParametricCurve>(std::make_shared<Arc>(
                              length / std::abs(angle_in_radians), angle_in_radians)),

    // Straight segment after the turn
    std::make_shared<Straight>(after_length)};

  const auto guide_curve = std::make_shared<CombinedCurve>(guide_curve_segments);

  const RoadCrossSectionDescription cross_section_description(
    2 * number_of_lanes_per_direction, width, number_of_lanes_per_direction);
  const RoadSegment segment(guide_curve, cross_section_description);

  const auto map = lanelet::utils::createMap(segment.getLanelets(resolution));

  map_fragment::write(*map, output_directory);

  std::cout << output_directory.c_str() << std::endl;

  return EXIT_SUCCESS;
} catch (const std::exception & exception) {
  std::cerr << exception.what() << std::endl;
  return EXIT_FAILURE;
}
