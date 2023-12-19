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

#ifndef MAP_FRAGMENT__ROAD_SEGMENTS__ROAD_SEGMENT__HPP_
#define MAP_FRAGMENT__ROAD_SEGMENTS__ROAD_SEGMENT__HPP_

#include <lanelet2_core/geometry/Lanelet.h>

#include <map_fragment/map_fragment.hpp>
#include <map_fragment/road_segments/parametric_curve.hpp>
#include <map_fragment/road_segments/road_cross_section_description.hpp>

namespace map_fragment::road_segments
{

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

using RoadSegments = std::vector<RoadSegment::ConstSharedPointer>;
}  // namespace map_fragment::road_segments

#endif  // MAP_FRAGMENT__ROAD_SEGMENTS__ROAD_SEGMENT__HPP_
