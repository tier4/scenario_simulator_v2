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
#include <map_fragment/road_segments/generate_lanelet_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>

enum IntersectionLeg { LEFT_LEG = 0, BOTTOM_LEG = 1, RIGHT_LEG = 2, TOP_LEG = 3 };

auto getLegOrientation(IntersectionLeg leg) -> double { return leg * M_PI_2; }

enum ConnectionType { LEFT_TURN = -1, FORWARD = 2, RIGHT_TURN = 1 };

auto getConnectedLeg(IntersectionLeg leg, ConnectionType connection_type) -> IntersectionLeg
{
  return static_cast<IntersectionLeg>(((leg + connection_type) + 4) % 4);
}

auto main(const int argc, char const * const * const argv) -> int
try {
  using namespace map_fragment;
  using namespace map_fragment::road_segments;

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

  const auto number_of_lanes_going_left = [&]() {
    node.declare_parameter("number_of_lanes_going_left", 0);
    return node.get_parameter("number_of_lanes_going_left").as_int();
  }();

  const auto number_of_lanes_going_left_and_straight = [&]() {
    node.declare_parameter("number_of_lanes_going_left_and_straight", 0);
    return node.get_parameter("number_of_lanes_going_left_and_straight").as_int();
  }();

  const auto number_of_lanes_going_straight = [&]() {
    node.declare_parameter("number_of_lanes_going_straight", 0);
    return node.get_parameter("number_of_lanes_going_straight").as_int();
  }();

  const auto number_of_lanes_going_in_all_directions = [&]() {
    node.declare_parameter("number_of_lanes_going_in_all_directions", 0);
    return node.get_parameter("number_of_lanes_going_in_all_directions").as_int();
  }();

  const auto number_of_lanes_going_straight_and_right = [&]() {
    node.declare_parameter("number_of_lanes_going_straight_and_right", 0);
    return node.get_parameter("number_of_lanes_going_straight_and_right").as_int();
  }();

  const auto number_of_lanes_going_right = [&]() {
    node.declare_parameter("number_of_lanes_going_right", 0);
    return node.get_parameter("number_of_lanes_going_right").as_int();
  }();

  const auto output_directory = [&]() {
    node.declare_parameter("output_directory", default_value::directory());
    return std::filesystem::path(node.get_parameter("output_directory").as_string());
  }();

  const auto total_number_of_lanes_per_leg_per_direction =
    number_of_lanes_going_left + number_of_lanes_going_left_and_straight +
    number_of_lanes_going_straight + number_of_lanes_going_in_all_directions +
    number_of_lanes_going_straight_and_right + number_of_lanes_going_right;

  const auto half_leg_width = total_number_of_lanes_per_leg_per_direction * lane_width;

  rcpputils::require_true(
    turn_radius >= half_leg_width,
    "Turn radius must not be smaller than half leg width. Actual values:\n"
    "- Turn radius: " +
      std::to_string(turn_radius) +
      "\n"
      "- Half leg width: " +
      std::to_string(half_leg_width));

  RoadSegments road_segments;
  RoadSegmentConnections road_segment_connections;

  const IntersectionLeg intersection_legs[] = {LEFT_LEG, BOTTOM_LEG, RIGHT_LEG, TOP_LEG};

  /*
   * Create road segments for each leg adjacent to the intersection
   */
  std::map<IntersectionLeg, RoadSegment::ConstSharedPointer> leg_segments;
  for (const auto leg : intersection_legs) {
    const auto leg_cross_section_description = RoadCrossSectionDescription(
      2 * total_number_of_lanes_per_leg_per_direction, lane_width,
      total_number_of_lanes_per_leg_per_direction);

    const auto leg_transformation = chainTransformations(
      makeTranslation(-(turn_radius + leg_length), 0, 0),
      makeRotationInZAxis(getLegOrientation(leg)));

    const auto leg_guide_curve =
      transformCurve(std::make_shared<Straight>(leg_length), leg_transformation);

    const auto leg_segment =
      std::make_shared<RoadSegment>(leg_guide_curve, leg_cross_section_description);

    leg_segments[leg] = leg_segment;
    road_segments.push_back(leg_segment);
  }

  /*
   * For each intersection leg...
   */
  for (const auto leg : intersection_legs) {
    const auto leg_segment = leg_segments[leg];

    const auto leg_boundary_transformation = chainTransformations(
      makeTranslation(-turn_radius, 0, 0), makeRotationInZAxis(getLegOrientation(leg)));

    /*
     * 1) Create connecting road segment for left-turning lanelets
     */

    const auto index_of_first_left_turning_lane = 0;

    const auto total_number_of_lanes_turning_left = number_of_lanes_going_left +
                                                    number_of_lanes_going_left_and_straight +
                                                    number_of_lanes_going_in_all_directions;

    if (total_number_of_lanes_turning_left > 0) {
      const auto [left_turn_cross_section_description, left_turn_lateral_offset] =
        generateSliceOfCrossSectionAndCalculateLateralOffset(
          leg_segment->cross_section_description, index_of_first_left_turning_lane,
          total_number_of_lanes_turning_left);

      const auto left_turn_guide_curve = transformCurve(
        shiftCurveLaterally(std::make_shared<Arc>(turn_radius, M_PI_2), left_turn_lateral_offset),
        leg_boundary_transformation);

      const auto left_turn_road_segment =
        std::make_shared<RoadSegment>(left_turn_guide_curve, left_turn_cross_section_description);
      road_segments.push_back(left_turn_road_segment);

      road_segment_connections.emplace_back(
        leg_segment, false, left_turn_road_segment, false, index_of_first_left_turning_lane);

      const auto connected_leg_left = getConnectedLeg(leg, ConnectionType::LEFT_TURN);
      const auto connected_leg_segment_left = leg_segments[connected_leg_left];

      road_segment_connections.emplace_back(
        left_turn_road_segment, false, connected_leg_segment_left, true,
        -index_of_first_left_turning_lane);
    }

    /*
     * 2) Create connecting road segment for straight-going lanelets
     */

    const auto index_of_first_straight_going_lane =
      index_of_first_left_turning_lane + number_of_lanes_going_left;

    const auto total_number_of_lanes_going_straight =
      number_of_lanes_going_left_and_straight + number_of_lanes_going_straight +
      number_of_lanes_going_in_all_directions + number_of_lanes_going_straight_and_right;

    if (total_number_of_lanes_going_straight > 0) {
      const auto [straight_cross_section_description, straight_lateral_offset] =
        generateSliceOfCrossSectionAndCalculateLateralOffset(
          leg_segment->cross_section_description, index_of_first_straight_going_lane,
          total_number_of_lanes_going_straight);

      const auto straight_guide_curve = transformCurve(
        shiftCurveLaterally(std::make_shared<Straight>(2 * turn_radius), straight_lateral_offset),
        leg_boundary_transformation);

      const auto straight_road_segment =
        std::make_shared<RoadSegment>(straight_guide_curve, straight_cross_section_description);
      road_segments.push_back(straight_road_segment);

      road_segment_connections.emplace_back(
        leg_segment, false, straight_road_segment, false, index_of_first_straight_going_lane);

      const auto connected_leg_straight = getConnectedLeg(leg, ConnectionType::FORWARD);
      const auto connected_leg_segment_straight = leg_segments[connected_leg_straight];

      road_segment_connections.emplace_back(
        straight_road_segment, false, connected_leg_segment_straight, true,
        -index_of_first_straight_going_lane);
    }

    /*
     * 3) Create connecting road segment for right-turning lanelets
     */

    const auto index_of_first_right_turning_lane =  //
      index_of_first_straight_going_lane + number_of_lanes_going_left_and_straight +
      number_of_lanes_going_straight;

    const auto total_number_of_lanes_turning_right = number_of_lanes_going_in_all_directions +
                                                     number_of_lanes_going_straight_and_right +
                                                     number_of_lanes_going_right;

    if (total_number_of_lanes_turning_right > 0) {
      const auto [right_turn_cross_section_description, right_turn_lateral_offset] =
        generateSliceOfCrossSectionAndCalculateLateralOffset(
          leg_segment->cross_section_description, index_of_first_right_turning_lane,
          total_number_of_lanes_turning_right);

      const auto right_turn_guide_curve = transformCurve(
        shiftCurveLaterally(std::make_shared<Arc>(turn_radius, -M_PI_2), right_turn_lateral_offset),
        leg_boundary_transformation);

      const auto right_turn_road_segment =
        std::make_shared<RoadSegment>(right_turn_guide_curve, right_turn_cross_section_description);
      road_segments.push_back(right_turn_road_segment);

      road_segment_connections.emplace_back(
        leg_segment, false, right_turn_road_segment, false, index_of_first_right_turning_lane);

      const auto connected_leg_right = getConnectedLeg(leg, ConnectionType::RIGHT_TURN);
      const auto connected_leg_segment_right = leg_segments[connected_leg_right];

      road_segment_connections.emplace_back(
        right_turn_road_segment, false, connected_leg_segment_right, true,
        -index_of_first_right_turning_lane);
    }
  }

  const auto map = generateLaneletMap(road_segments, resolution, road_segment_connections);
  map_fragment::write(*map, output_directory);

  std::cout << output_directory.c_str() << std::endl;

  return EXIT_SUCCESS;
} catch (const std::exception & exception) {
  std::cerr << exception.what() << std::endl;
  return EXIT_FAILURE;
}
