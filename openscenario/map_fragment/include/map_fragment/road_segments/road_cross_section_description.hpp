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

#ifndef MAP_FRAGMENT__ROAD_SEGMENTS__ROAD_CROSS_SECTION_DESCRIPTION__HPP_
#define MAP_FRAGMENT__ROAD_SEGMENTS__ROAD_CROSS_SECTION_DESCRIPTION__HPP_

#include <rcpputils/asserts.hpp>

namespace map_fragment::road_segments
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
}  // namespace map_fragment::road_segments

#endif  // MAP_FRAGMENT__ROAD_SEGMENTS__ROAD_CROSS_SECTION_DESCRIPTION__HPP_