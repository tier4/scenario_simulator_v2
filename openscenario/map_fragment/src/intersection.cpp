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

// Structure of the intersection
// TODO: Make this configurable
const uint64_t NUMBER_OF_LANES_LEFT = 0;
const uint64_t NUMBER_OF_LANES_LEFT_OR_STRAIGHT = 0;
const uint64_t NUMBER_OF_LANES_STRAIGHT = 0;
const uint64_t NUMBER_OF_LANES_ANY_DIRECTION = 2;
const uint64_t NUMBER_OF_LANES_STRAIGHT_OR_RIGHT = 0;
const uint64_t NUMBER_OF_LANES_RIGHT = 0;

auto main(const int argc, char const * const * const argv) -> int
try {
  using namespace map_fragment;

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node(std::filesystem::path(argv[0]).stem());

  const auto lane_width = [&]() {
    node.declare_parameter("lane_width", default_value::width);
    return node.get_parameter("lane_width").as_double();
  }();

  const auto leg_length = [&]() {
    node.declare_parameter("leg_length", default_value::length);
    return node.get_parameter("leg_length").as_double();
  }();

  const auto turn_radius = [&]() {
    node.declare_parameter("turn_radius", default_value::turn_radius);
    return node.get_parameter("turn_radius").as_double();
  }();

  const auto resolution = [&]() {
    node.declare_parameter("resolution", default_value::resolution);
    return node.get_parameter("resolution").as_int();
  }();

  const auto output_directory = [&]() {
    node.declare_parameter("output_directory", default_value::directory());
    return std::filesystem::path(node.get_parameter("output_directory").as_string());
  }();

  std::vector<RoadSegment> road_segments;
  auto total_number_of_lanes_per_leg_per_direction =
    NUMBER_OF_LANES_LEFT + NUMBER_OF_LANES_LEFT_OR_STRAIGHT + NUMBER_OF_LANES_STRAIGHT +
    NUMBER_OF_LANES_ANY_DIRECTION + NUMBER_OF_LANES_STRAIGHT_OR_RIGHT + NUMBER_OF_LANES_RIGHT;

  /*
   * For each intersection leg...
   */
  for (auto i = 0; i < 4; i++) {
    const auto leg_cross_section_description =
      RoadCrossSectionDescription(2 * total_number_of_lanes_per_leg_per_direction, lane_width);

    const auto leg_transformation =
      chainTransformations(makeTranslation(-turn_radius, 0, 0), makeRotationInZAxis(i * M_PI_2));

    /*
     * 1) Create road segment for the leg (adjacent to the intersection)
     */

    const auto leg_guide_curve = transformCurve(
      std::make_shared<Straight>(leg_length),
      chainTransformations(makeTranslation(-leg_length, 0, 0), leg_transformation));

    road_segments.emplace_back(leg_guide_curve, leg_cross_section_description);

    /*
     * 2) Create connecting road segment for left-turning lanelets
     */

    const auto total_number_of_lanes_turning_left =
      NUMBER_OF_LANES_LEFT + NUMBER_OF_LANES_LEFT_OR_STRAIGHT + NUMBER_OF_LANES_ANY_DIRECTION;

    const auto [left_turn_cross_section_description, left_turn_lateral_offset] =
      generateSliceOfCrossSectionAndCalculateLateralOffset(
        leg_cross_section_description, 0, total_number_of_lanes_turning_left);

    const auto left_turn_guide_curve = transformCurve(
      shiftCurveLaterally(std::make_shared<Arc>(turn_radius, M_PI_2), left_turn_lateral_offset),
      leg_transformation);

    road_segments.emplace_back(left_turn_guide_curve, left_turn_cross_section_description);

    /*
     * 3) Create connecting road segment for straight-going lanelets
     */

    const auto index_of_first_straight_going_lane = NUMBER_OF_LANES_LEFT;

    const auto total_number_of_lanes_going_straight =
      NUMBER_OF_LANES_LEFT_OR_STRAIGHT + NUMBER_OF_LANES_STRAIGHT + NUMBER_OF_LANES_ANY_DIRECTION +
      NUMBER_OF_LANES_STRAIGHT_OR_RIGHT;

    const auto [straight_cross_section_description, straight_lateral_offset] =
      generateSliceOfCrossSectionAndCalculateLateralOffset(
        leg_cross_section_description, index_of_first_straight_going_lane,
        total_number_of_lanes_going_straight);

    const auto straight_guide_curve = transformCurve(
      shiftCurveLaterally(std::make_shared<Straight>(2 * turn_radius), straight_lateral_offset),
      leg_transformation);

    road_segments.emplace_back(straight_guide_curve, straight_cross_section_description);

    /*
     * 4) Create connecting road segment for right-turning lanelets
     */

    const auto index_of_first_right_turning_lane =  //
      index_of_first_straight_going_lane + NUMBER_OF_LANES_LEFT_OR_STRAIGHT +
      NUMBER_OF_LANES_STRAIGHT;

    const auto total_number_of_lanes_turning_right =
      NUMBER_OF_LANES_ANY_DIRECTION + NUMBER_OF_LANES_STRAIGHT_OR_RIGHT + NUMBER_OF_LANES_RIGHT;

    const auto [right_turn_cross_section_description, right_turn_lateral_offset] =
      generateSliceOfCrossSectionAndCalculateLateralOffset(
        leg_cross_section_description, index_of_first_right_turning_lane,
        total_number_of_lanes_turning_right);

    const auto right_turn_guide_curve = transformCurve(
      shiftCurveLaterally(std::make_shared<Arc>(turn_radius, -M_PI_2), right_turn_lateral_offset),
      leg_transformation);

    road_segments.emplace_back(right_turn_guide_curve, right_turn_cross_section_description);
  }

  lanelet::Lanelets lanelets;
  for (auto & road_segment : road_segments) {
    const auto road_segment_lanelets = road_segment.getLanelets(resolution);

    lanelets.insert(lanelets.end(), road_segment_lanelets.begin(), road_segment_lanelets.end());
  }

  const auto map = lanelet::utils::createMap(lanelets);

  map_fragment::write(*map, output_directory);

  std::cout << output_directory.c_str() << std::endl;

  return EXIT_SUCCESS;
} catch (const std::exception & exception) {
  std::cerr << exception.what() << std::endl;
  return EXIT_FAILURE;
}