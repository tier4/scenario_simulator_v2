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

#include <boost/geometry.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <exception>
#include <iostream>
#include <numeric>

namespace entity_behavior::vehicle::follow_lane_sequence
{
namespace
{
namespace bg = boost::geometry;
using BoostPoint = bg::model::d2::point_xy<double>;
using BoostPolygon = bg::model::polygon<BoostPoint>;
constexpr char kFrameId[] = "map";
constexpr double kFillAlpha = 0.4;
constexpr double kOutlineAlpha = 0.9;
constexpr double kOutlineWidth = 0.1;
constexpr double kLifetimeSeconds = 1.0;

auto makeTriangleMarker(
  const std::string & ns, int32_t id, const rclcpp::Time & stamp,
  const std::vector<geometry_msgs::msg::Point> & points) -> visualization_msgs::msg::Marker
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = kFrameId;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 0.0F;
  marker.color.g = 0.6F;
  marker.color.b = 1.0F;
  marker.color.a = kFillAlpha;
  marker.points = points;
  marker.colors = std::vector<std_msgs::msg::ColorRGBA>(points.size(), marker.color);
  marker.lifetime = rclcpp::Duration::from_seconds(kLifetimeSeconds);
  return marker;
}

auto makeOutlineMarker(
  const std::string & ns, int32_t id, const rclcpp::Time & stamp, const double z_offset,
  const std::vector<BoostPolygon> & polygons) -> visualization_msgs::msg::Marker
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = kFrameId;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = kOutlineWidth;
  marker.color.r = 1.0F;
  marker.color.g = 0.2F;
  marker.color.b = 0.0F;
  marker.color.a = kOutlineAlpha;

  for (const auto & polygon : polygons) {
    const auto & outer = polygon.outer();
    if (outer.size() < 2) {
      continue;
    }
    for (std::size_t i = 0; i + 1 < outer.size(); ++i) {
      geometry_msgs::msg::Point from;
      from.x = outer[i].x();
      from.y = outer[i].y();
      from.z = z_offset;
      geometry_msgs::msg::Point to;
      to.x = outer[i + 1].x();
      to.y = outer[i + 1].y();
      to.z = z_offset;
      marker.points.push_back(from);
      marker.points.push_back(to);
    }
  }
  marker.lifetime = rclcpp::Duration::from_seconds(kLifetimeSeconds);
  return marker;
}

auto makeDeleteMarker(const std::string & ns, int32_t id) -> visualization_msgs::msg::Marker
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = kFrameId;
  marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  marker.ns = ns;
  marker.id = id;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  return marker;
}
}  // namespace

auto logSplineDebugInfo(
  const std::string & action_name,
  const traffic_simulator_msgs::msg::WaypointsArray & waypoints,
  const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> &
    canonicalized_entity_status) -> std::vector<visualization_msgs::msg::Marker>
{
  std::vector<visualization_msgs::msg::Marker> markers;
  const auto entity_name =
    canonicalized_entity_status ? canonicalized_entity_status->getName() : std::string("unknown");
  const auto ns = action_name + "_" + entity_name;
  try {
    math::geometry::CatmullRomSpline debug_spline(waypoints.waypoints);
    const auto debug_polygon = debug_spline.getPolygon(1.0, 10);

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

    const auto stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    if (debug_polygon.empty()) {
      markers.emplace_back(makeDeleteMarker(ns, 0));
      markers.emplace_back(makeDeleteMarker(ns, 1));
      return markers;
    }

    const double average_z = std::accumulate(
                               debug_polygon.begin(), debug_polygon.end(), 0.0,
                               [](double sum, const auto & point) { return sum + point.z; }) /
                             static_cast<double>(debug_polygon.size());

    markers.emplace_back(makeTriangleMarker(ns, 0, stamp, debug_polygon));
    markers.emplace_back(makeOutlineMarker(ns, 1, stamp, average_z, boost_polygons));
  } catch (const std::exception & e) {
    std::cout << "[" << action_name << "] spline debug error: " << e.what() << std::endl;
  }
  return markers;
}
}  // namespace entity_behavior::vehicle::follow_lane_sequence
