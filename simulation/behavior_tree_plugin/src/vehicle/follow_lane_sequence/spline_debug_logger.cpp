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

#include <behavior_tree_plugin/vehicle/follow_lane_sequence/spline_debug_logger.hpp>

#include <geometry/spline/catmull_rom_spline.hpp>
#include <traffic_simulator/data_type/entity_status.hpp>

#include <boost/geometry.hpp>

#include <exception>
#include <iostream>
#include <vector>

namespace entity_behavior::vehicle::follow_lane_sequence
{
void logSplineDebugInfo(
  const std::string & action_name,
  const traffic_simulator_msgs::msg::WaypointsArray & waypoints,
  const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> &
    canonicalized_entity_status)
{
  const auto entity_name =
      canonicalized_entity_status ? canonicalized_entity_status->getName() : std::string("unknown");
  if (entity_name != "Bicycle0") { return; }
  try {
    math::geometry::CatmullRomSpline debug_spline(waypoints.waypoints);
    const auto debug_polygon = debug_spline.getPolygon(1.0, 10);
    namespace bg = boost::geometry;
    using BoostPoint = bg::model::d2::point_xy<double>;
    using BoostPolygon = bg::model::polygon<BoostPoint>;
    std::vector<BoostPolygon> boost_polygons;
    boost_polygons.reserve(debug_polygon.size() / 3);
    for (std::size_t index = 0; index + 2 < debug_polygon.size(); index += 3) {
      BoostPolygon polygon;
      auto & outer = polygon.outer();
      outer.emplace_back(debug_polygon[index].x, debug_polygon[index].y);
      outer.emplace_back(debug_polygon[index + 1].x, debug_polygon[index + 1].y);
      outer.emplace_back(debug_polygon[index + 2].x, debug_polygon[index + 2].y);
      outer.push_back(outer.front());
      bg::correct(polygon);
      boost_polygons.push_back(polygon);
    }

    std::cout << "[" << action_name << "] [" << entity_name << "] debug polygon size="
              << debug_polygon.size() << " boost polygons=" << boost_polygons.size();
    if (!debug_polygon.empty()) {
      const auto & first_point = debug_polygon.front();
      const auto & last_point = debug_polygon.back();
      std::cout << " first=(" << first_point.x << ", " << first_point.y << ", " << first_point.z
                << ")";
      std::cout << " last=(" << last_point.x << ", " << last_point.y << ", " << last_point.z
                << ")";
    }
    std::cout << std::endl;
  } catch (const std::exception & e) {
    std::cout << "[" << action_name << "] spline debug error: " << e.what() << std::endl;
  }
}
}  // namespace entity_behavior::vehicle::follow_lane_sequence
