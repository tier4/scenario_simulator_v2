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
#include <map_fragment/geometry.hpp>

namespace map_fragment
{
struct RoadCrossSectionDescription
{
  const int number_of_lanes;
  const double lane_width;

  explicit RoadCrossSectionDescription(int number_of_lanes,
                                       double lane_width)
  : number_of_lanes(number_of_lanes)
  , lane_width(lane_width)
  {
    if (number_of_lanes <= 0)
    {
      throw std::invalid_argument(
        "Expected number_of_lanes to be positive. Actual value: "
        + std::to_string(number_of_lanes)
      );
    }

    if (lane_width <= 0.0)
    {
      throw std::invalid_argument(
        "Expected lane_width to be positive. Actual value: "
        + std::to_string(lane_width)
      );
    }
  }
}; // struct RoadCrossSectionDescription

class RoadCrossSection
{
  lanelet::Points3d points_;

public:
  explicit RoadCrossSection(RoadCrossSectionDescription const& description,
                            Point2d const& origin,
                            Vector2d const& tangent_vector)
  {
    auto n = description.number_of_lanes;
    auto w = description.lane_width;
    auto normal_vector = rotate(tangent_vector, M_PI / 2);

    for (auto i = 0; i < description.number_of_lanes + 1; i++)
    {
      auto lateral_position = (i - n / 2) * w;
      auto p = origin + normal_vector * lateral_position;
      points_.push_back(makePoint3d(p.x, p.y, 0.));

    }
  }

  const lanelet::Points3d getPoints()
  { // TODO: return as smart pointer?
    return points_;
  }
}; // class RoadCrossSection

class RoadSegment
{
  ParametricCurve::Ptr guide_curve_;
  RoadCrossSectionDescription cross_section_description_;

public:
  explicit RoadSegment(ParametricCurve::Ptr guide_curve,
                       RoadCrossSectionDescription const& cross_section_description)
    : guide_curve_(guide_curve)
    , cross_section_description_(cross_section_description)
  {
  }

  const lanelet::Lanelets getLanelets(double resolution)
  {
    lanelet::Lanelets lanelets;
    auto n = cross_section_description_.number_of_lanes + 1;

    // TODO Find a cleaner way to do this
    std::vector<lanelet::LineString3d> lane_boundaries(n);
    for (auto i = 0; i < n; i++)
    {
      lane_boundaries[i].setId(i);
    }

    for (auto i = 0; i < resolution; i++)
    {
      auto t = i / (resolution - 1);
      auto position = guide_curve_->getPosition(t);
      auto tangent_vector = guide_curve_->getTangentVector(t);

      RoadCrossSection cross_section(cross_section_description_, position, tangent_vector);
      auto cross_section_points = cross_section.getPoints();

      for (auto j = 0; j < cross_section_description_.number_of_lanes + 1; j++)
      {
        lane_boundaries[j].push_back(cross_section_points[j]);
      }
    }

    for (auto i = 0; i < cross_section_description_.number_of_lanes; i++)
    {
      lanelets.push_back(makeLanelet(
        lane_boundaries[i],
        lane_boundaries[i + 1]));
    }

    return lanelets;
  }
}; // class RoadSegment

} // namespace map_fragment

#endif // MAP_FRAGMENT__ROAD_SEGMENT__HPP_