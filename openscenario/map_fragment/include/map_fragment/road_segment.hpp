// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MAP_FRAGMENT__ROAD_SEGMENT__HPP_
#define MAP_FRAGMENT__ROAD_SEGMENT__HPP_

#include <lanelet2_core/geometry/Lanelet.h>

#include <map_fragment/map_fragment.hpp>
#include <map_fragment/parametric_curve.hpp>
#include <rcpputils/asserts.hpp>

namespace map_fragment
{

/**
 * Description of a road cross section, defining its number of lanes and lane width
 */
struct RoadCrossSectionDescription
{
  const int number_of_lanes;

  const double lane_width;

  const int number_of_reversed_lanes;

  explicit RoadCrossSectionDescription(
    const int number_of_lanes, const double lane_width, const int number_of_reversed_lanes = 0)
  : number_of_lanes(number_of_lanes),
    lane_width(lane_width),
    number_of_reversed_lanes(number_of_reversed_lanes)
  {
    rcpputils::require_true(
      number_of_lanes > 0,
      "Expected number_of_lanes to be positive. Actual value: " + std::to_string(number_of_lanes));

    rcpputils::require_true(
      lane_width > 0.0,
      "Expected lane_width to be positive. Actual value: " + std::to_string(lane_width));

    rcpputils::require_true(
      number_of_reversed_lanes >= 0, "Expected lane_width to be non-negative. Actual value: " +
                                       std::to_string(number_of_reversed_lanes));
  }
};  // struct RoadCrossSectionDescription

/**
 * Road segment, defined as the result of sliding a road cross section across a guide curve
 */
struct RoadSegment
{
public:
  using ConstSharedPointer = std::shared_ptr<const RoadSegment>;

  const ParametricCurve::ConstSharedPointer guide_curve;

  const RoadCrossSectionDescription cross_section_description;

  explicit RoadSegment(
    const ParametricCurve::ConstSharedPointer guide_curve,
    const RoadCrossSectionDescription & cross_section_description)
  : guide_curve(guide_curve), cross_section_description(cross_section_description)
  {
  }

  explicit RoadSegment(const RoadSegment &) = delete;
};  // class RoadSegment

/**
 * Make a road segment with constant curvature
 */
auto makeCurvedRoadSegment(
  const double curvature, const double length, const int number_of_lanes, const double lane_width)
  -> RoadSegment::ConstSharedPointer
{
  const auto guide_curve = curvature == 0.0
                             ? std::static_pointer_cast<ParametricCurve>(  //
                                 std::make_shared<Straight>(length))
                             : std::static_pointer_cast<ParametricCurve>(std::make_shared<Arc>(
                                 std::abs(1 / curvature), length * curvature));

  const RoadCrossSectionDescription cross_section_description(number_of_lanes, lane_width);

  return std::make_shared<RoadSegment>(guide_curve, cross_section_description);
}

auto generateSliceOfCrossSectionAndCalculateLateralOffset(
  const RoadCrossSectionDescription & cross_section_description, const int & first_lane_index,
  const int & number_of_lanes) -> std::pair<RoadCrossSectionDescription, double>
{
  const auto slice_description =
    RoadCrossSectionDescription(number_of_lanes, cross_section_description.lane_width);

  const auto lateral_offset =
    -cross_section_description.lane_width *
    (first_lane_index + 0.5 * number_of_lanes - 0.5 * cross_section_description.number_of_lanes);

  return std::make_pair(slice_description, lateral_offset);
}

using RoadSegments = std::vector<RoadSegment::ConstSharedPointer>;

/**
 * Sequential connection between two subsequent road segments
 *
 * Nominally, end of the first road segment is connected to the end of the second one.
 * For different behavior, either of the road segments can be inverted to swap
 * its beginning with its end.
 *
 * To define which lanes of the first road segment connect to which lanes
 * of the second one, a lateral offset is specified. Without an offset,
 * leftmost lanes of both segments are aligned. Positive offset means
 * the second segment is shifted to the right by the specified number of lanes.
 * Conversely, a negative offset means the first segment is shifted to the right
 * (which is equivalent to the second one being shifted to the left).
 */
struct RoadSegmentConnection
{
  const RoadSegment::ConstSharedPointer first_segment;

  const bool is_first_segment_inverted;

  const RoadSegment::ConstSharedPointer second_segment;

  const bool is_second_segment_inverted;

  const int offset;

  explicit RoadSegmentConnection(
    const RoadSegment::ConstSharedPointer first_segment, const bool is_first_segment_inverted,
    const RoadSegment::ConstSharedPointer second_segment, const bool is_second_segment_inverted,
    const int offset)
  : first_segment(first_segment),
    is_first_segment_inverted(is_first_segment_inverted),
    second_segment(second_segment),
    is_second_segment_inverted(is_second_segment_inverted),
    offset(offset)
  {
    rcpputils::require_true(
      first_segment->cross_section_description.lane_width ==
        second_segment->cross_section_description.lane_width,
      "Connected segments must have equal lane width");

    const auto lane_width = first_segment->cross_section_description.lane_width;

    rcpputils::require_true(
      getNumberOfOverlappingLanes() > 0,
      "Connected segments must have at least one overlapping lane");

    const auto first_segment_guide_curve_parameter_at_connection =
      is_first_segment_inverted ? 0 : 1;
    const auto second_segment_guide_curve_parameter_at_connection =
      is_second_segment_inverted ? 1 : 0;

    const auto first_segment_guide_curve_position_at_connection =
      first_segment->guide_curve->getPosition(first_segment_guide_curve_parameter_at_connection);
    const auto second_segment_guide_curve_position_at_connection =
      second_segment->guide_curve->getPosition(second_segment_guide_curve_parameter_at_connection);

    const auto first_segment_tangent_vector_at_connection = Vector(
      first_segment->guide_curve->getUnitTangentVector(
        first_segment_guide_curve_parameter_at_connection) *
      (is_first_segment_inverted ? -1 : 1));
    const auto second_segment_tangent_vector_at_connection = Vector(
      second_segment->guide_curve->getUnitTangentVector(
        second_segment_guide_curve_parameter_at_connection) *
      (is_second_segment_inverted ? -1 : 1));

    rcpputils::require_true(
      areUnitVectorsInTheSameDirection(
        first_segment_tangent_vector_at_connection, second_segment_tangent_vector_at_connection),
      "At the connection point, tangent vectors of connected segments' guide curves must be in the "
      "same direction (after accounting for inverted segments)");

    const auto first_segment_normal_vector_at_connection =
      rotateInLocalZAxisAssumingZeroRoll(first_segment_tangent_vector_at_connection, M_PI_2);
    const auto second_segment_normal_vector_at_connection =
      rotateInLocalZAxisAssumingZeroRoll(second_segment_tangent_vector_at_connection, M_PI_2);

    const auto first_segment_leftmost_boundary_connection_point =
      first_segment_guide_curve_position_at_connection +
      (first_segment->cross_section_description.number_of_lanes / 2. - getSecondSegmentOffset()) *
        lane_width * first_segment_normal_vector_at_connection;

    const auto second_segment_leftmost_boundary_connection_point =
      second_segment_guide_curve_position_at_connection +
      (second_segment->cross_section_description.number_of_lanes / 2. - getFirstSegmentOffset()) *
        lane_width * second_segment_normal_vector_at_connection;

    rcpputils::require_true(
      doPointsOverlap(
        first_segment_leftmost_boundary_connection_point,
        second_segment_leftmost_boundary_connection_point),
      "Respective ends of connected boundaries should overlap");
  }

  /*
   * Calculate by how many lanes should the first segment be shifted to the right
   */
  auto getFirstSegmentOffset() const -> int { return std::max(0, -offset); }

  /*
   * Calculate by how many lanes should the second segment be shifted to the right
   */
  auto getSecondSegmentOffset() const -> int { return std::max(0, offset); }

  /*
   * Calculate how many lanes overlap between the two segments
   */
  auto getNumberOfOverlappingLanes() const -> int
  {
    return std::min(
      first_segment->cross_section_description.number_of_lanes - getSecondSegmentOffset(),
      second_segment->cross_section_description.number_of_lanes - getFirstSegmentOffset());
  }

  /**
   * Connection between a lane boundary of the first segment
   * and a lane boundary of the second segment.
   *
   * Represented as a pair of indices, where 0 means
   * the left-most boundary for the given road segment.
   */
  using BoundarywiseConnection = std::pair<int, int>;
  using BoundarywiseConnections = std::vector<BoundarywiseConnection>;

  /*
   * Calculate lane-boundary-wise connections
   */
  auto getBoundarywiseConnections() const -> BoundarywiseConnections
  {
    BoundarywiseConnections connections;

    const auto number_of_connections = getNumberOfOverlappingLanes() + 1;
    const auto first_segment_offset = getFirstSegmentOffset();
    const auto second_segment_offset = getSecondSegmentOffset();

    for (auto i = 0; i < number_of_connections; i++) {
      const auto first_point_index =
        is_first_segment_inverted
          ? first_segment->cross_section_description.number_of_lanes - i - second_segment_offset
          : i + second_segment_offset;

      const auto second_point_index =
        is_second_segment_inverted
          ? second_segment->cross_section_description.number_of_lanes - i - first_segment_offset
          : i + first_segment_offset;

      connections.emplace_back(first_point_index, second_point_index);
    }

    return connections;
  }
};  // struct RoadSegmentConnection

using RoadSegmentConnections = std::vector<RoadSegmentConnection>;
}  // namespace map_fragment

#endif  // MAP_FRAGMENT__ROAD_SEGMENT__HPP_
