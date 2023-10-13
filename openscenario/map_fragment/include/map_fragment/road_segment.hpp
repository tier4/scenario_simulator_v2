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

namespace map_fragment
{
struct RoadCrossSectionDescription
{
  long int number_of_lanes;
  double lane_width;
}; // struct RoadCrossSectionDescription

class RoadCrossSection
{
  lanelet::Points3d points_;

public:
  RoadCrossSection(RoadCrossSectionDescription description, // TODO: name for the origin coordinate system
                   double origin_x,
                   double origin_y,
                   double h)
  {

    auto cross_section_width = description.number_of_lanes * description.lane_width;

    auto x = origin_x + cross_section_width / 2 * std::sin(h);
    auto y = origin_y - cross_section_width / 2 * std::cos(h);

    for (auto i = 0; i < description.number_of_lanes + 1; i++)
    {
      points_.push_back(makePoint3d(x, y, 0.));
      x -= description.lane_width * std::sin(h);
      y += description.lane_width * std::cos(h);
    }
  }

  const lanelet::Points3d getPoints()
  { // TODO: return as smart pointer?
    return points_;
  }
}; // class RoadCrossSection

class RoadSegment
{
  lanelet::Lanelets lanelets_;

public:
  RoadSegment(double length,
              double curvature,
              int resolution,
              RoadCrossSectionDescription cross_section_description)
  {

    //  TODO Make sure resolution >= 2
    //  TODO Make sure length > 0

    auto step = length / (resolution - 1);

    // TODO Allow to initialize origin with custom values
    auto x = 0.0;
    auto y = 0.0;
    auto h = 0.0;

    auto n = cross_section_description.number_of_lanes + 1;

    // TODO Find a cleaner way to do this
    std::vector<lanelet::LineString3d> lane_boundaries(n);
    for (auto i = 0; i < n; i++)
    {
      lane_boundaries[i].setId(i);
    }

    for (auto i = 0; i < resolution; i++)
    {
      RoadCrossSection cross_section(cross_section_description, x, y, h);
      auto cross_section_points = cross_section.getPoints();

      for (auto j = 0; j < cross_section_description.number_of_lanes + 1; j++)
      {
        lane_boundaries[j].push_back(cross_section_points[j]);
      }

      x += step * std::cos(h);
      y += step * std::sin(h);
      h += step * curvature;
    }

    for (auto i = 0; i < cross_section_description.number_of_lanes; i++)
    {
      lanelets_.push_back(makeLanelet(
        lane_boundaries[i],
        lane_boundaries[i + 1]));
    }
  }

  const lanelet::Lanelets getLanelets()
  { // TODO: return as smart pointer?
    return lanelets_;
  }
}; // class RoadSegment

} // namespace map_fragment

#endif // MAP_FRAGMENT__ROAD_SEGMENT__HPP_