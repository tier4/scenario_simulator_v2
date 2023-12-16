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
 * Cross section of the road
 */
class RoadCrossSection
{
  lanelet::Points3d points_;

public:
  explicit RoadCrossSection(
    const RoadCrossSectionDescription & description, const Point & origin,
    const Vector & tangent_vector)
  {
    const auto normal_vector = rotateInLocalZAxisAssumingZeroRoll(tangent_vector, M_PI_2);

    for (auto i = 0; i < description.number_of_lanes + 1; i++) {
      const auto lateral_position = (description.number_of_lanes / 2. - i) * description.lane_width;
      const auto p = origin + normal_vector * lateral_position;

      points_.push_back(makePoint3d(p));
    }
  }

  /**
   * Get points belonging to lane boundaries at the cross section, left to right
   */
  auto getPoints() const -> const lanelet::Points3d & { return points_; }
};  // class RoadCrossSection

/**
 * Road segment, defined as the result of sliding a road cross section across a guide curve
 */
class RoadSegment
{
public:
  const ParametricCurve::ConstSharedPointer guide_curve;

  const RoadCrossSectionDescription cross_section_description;

  explicit RoadSegment(
    const ParametricCurve::ConstSharedPointer guide_curve,
    const RoadCrossSectionDescription & cross_section_description)
  : guide_curve(guide_curve), cross_section_description(cross_section_description)
  {
  }

  /**
   * Get lanelets which compose the road segment, left to right
   */
  auto getLanelets(double resolution) const -> lanelet::Lanelets
  {
    lanelet::Lanelets lanelets;
    const auto number_of_lane_boundaries = cross_section_description.number_of_lanes + 1;
    const auto index_of_first_reversed_lane = cross_section_description.number_of_lanes -
                                              cross_section_description.number_of_reversed_lanes;

    std::vector<lanelet::LineString3d> lane_boundaries(number_of_lane_boundaries);
    for (auto boundary : lane_boundaries) {
      boundary.setId(generateNextLineStringId());
    }

    for (auto i = 0; i < resolution; i++) {
      const auto tangent = i / (resolution - 1);
      const auto position = guide_curve->getPosition(tangent);
      const auto tangent_vector = guide_curve->getUnitTangentVector(tangent);

      const RoadCrossSection cross_section(cross_section_description, position, tangent_vector);
      const auto cross_section_points = cross_section.getPoints();

      for (auto j = 0; j < cross_section_description.number_of_lanes + 1; j++) {
        lane_boundaries[j].push_back(cross_section_points[j]);
      }
    }

    for (auto i = 0; i < cross_section_description.number_of_lanes; i++) {
      lanelets.push_back(
        i >= index_of_first_reversed_lane
          ? makeLanelet(lane_boundaries[i + 1], lane_boundaries[i])
          : makeLanelet(lane_boundaries[i], lane_boundaries[i + 1]));
    }

    return lanelets;
  }
};  // class RoadSegment

/**
 * Make a road segment with constant curvature
 */
auto makeCurvedRoadSegment(
  const double curvature, const double length, const int number_of_lanes, const double lane_width)
  -> RoadSegment
{
  const auto guide_curve = curvature == 0.0
                             ? std::static_pointer_cast<ParametricCurve>(  //
                                 std::make_shared<Straight>(length))
                             : std::static_pointer_cast<ParametricCurve>(std::make_shared<Arc>(
                                 std::abs(1 / curvature), length * curvature));

  const RoadCrossSectionDescription cross_section_description(number_of_lanes, lane_width);

  return RoadSegment(guide_curve, cross_section_description);
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
}  // namespace map_fragment

#endif  // MAP_FRAGMENT__ROAD_SEGMENT__HPP_
