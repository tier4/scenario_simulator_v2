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

#ifndef BEHAVIOR_TREE_PLUGIN__VEHICLE__FOLLOW_LANE_SEQUENCE__SPLINE_DEBUG_CALCULATOR_HPP_
#define BEHAVIOR_TREE_PLUGIN__VEHICLE__FOLLOW_LANE_SEQUENCE__SPLINE_DEBUG_CALCULATOR_HPP_

#include <array>
#include <string>
#include <utility>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/point.hpp>

#include <traffic_simulator/data_type/entity_status.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace math::geometry
{
class CatmullRomSpline;
}  // namespace math::geometry

namespace entity_behavior::vehicle::follow_lane_sequence
{
namespace bg = boost::geometry;
using BoostPoint = bg::model::d2::point_xy<double>;
using BoostPolygon = bg::model::polygon<BoostPoint>;

struct QuadrilateralData
{
  std::vector<std::array<geometry_msgs::msg::Point, 4>> quadrilaterals;
  std::vector<BoostPolygon> polygons;
  std::vector<std::pair<double, double>> longitudinal_ranges;
};

struct EntityCollisionInfo
{
  std::string name;
  const traffic_simulator::CanonicalizedEntityStatus * status;
  BoostPolygon polygon;
  bool intersects_trajectory;
};

auto buildQuadrilateralData(
  const math::geometry::CatmullRomSpline & spline, double width, std::size_t num_segments)
  -> QuadrilateralData;

auto detectEntityCollisions(
  const QuadrilateralData & data,
  const std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus> &
    other_entity_status,
  const std::string & entity_name) -> std::vector<EntityCollisionInfo>;
}  // namespace entity_behavior::vehicle::follow_lane_sequence

#endif  // BEHAVIOR_TREE_PLUGIN__VEHICLE__FOLLOW_LANE_SEQUENCE__SPLINE_DEBUG_CALCULATOR_HPP_
