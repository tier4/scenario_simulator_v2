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

#include <geometry/bounding_box.hpp>
#include <geometry/spline/catmull_rom_spline.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <chrono>

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <exception>
#include <iostream>
#include <mutex>
#include <numeric>
#include <unordered_map>

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
constexpr double kLifetimeSeconds = 0.0;

struct ImmediateMarkerPublisher
{
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher;
};

auto getImmediatePublisher() -> ImmediateMarkerPublisher &
{
  static ImmediateMarkerPublisher handles;
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);
  if (!handles.publisher) {
    try {
      handles.node = rclcpp::Node::make_shared("spline_debug_logger_immediate");
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
      handles.publisher = handles.node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/simulation/spline_debug_polygon", qos);
    } catch (const std::exception & e) {
      std::cerr << "[spline_debug_logger] failed to create immediate marker publisher: " << e.what()
                << std::endl;
      handles.node.reset();
      handles.publisher.reset();
    }
  }
  return handles;
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

auto publishImmediate(const std::vector<visualization_msgs::msg::Marker> & markers) -> void
{
  auto & handles = getImmediatePublisher();
  if (!handles.publisher) {
    return;
  }
  visualization_msgs::msg::MarkerArray array_msg;
  array_msg.markers = markers;
  handles.publisher->publish(array_msg);
}
}  // namespace

auto logSplineDebugInfo(
  const std::string & action_name,
  const traffic_simulator_msgs::msg::WaypointsArray & waypoints,
  const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> &
    canonicalized_entity_status,
  const std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus> &
    other_entity_status) -> std::vector<visualization_msgs::msg::Marker>
{
  const auto start = std::chrono::steady_clock::now();
  std::vector<visualization_msgs::msg::Marker> markers;
  const auto entity_name =
    canonicalized_entity_status ? canonicalized_entity_status->getName() : std::string("unknown");
  const auto ns = action_name + "_" + entity_name;
  try {
    math::geometry::CatmullRomSpline debug_spline(waypoints.waypoints);
    const auto debug_polygon = debug_spline.getPolygon(1.0, 30);

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
    } else {
      const double average_z = std::accumulate(
                                 debug_polygon.begin(), debug_polygon.end(), 0.0,
                                 [](double sum, const auto & point) { return sum + point.z; }) /
                               static_cast<double>(debug_polygon.size());

      markers.emplace_back(makeOutlineMarker(ns, 1, stamp, average_z, boost_polygons));

      std::vector<std::pair<std::string, const traffic_simulator::CanonicalizedEntityStatus *>>
        entity_statuses;
      const auto own_status_ptr = canonicalized_entity_status ? canonicalized_entity_status.get() : nullptr;
      for (const auto & [name, status] : other_entity_status) {
        entity_statuses.emplace_back(name, &status);
      }
      if (own_status_ptr != nullptr) {
        entity_statuses.emplace_back(entity_name, own_status_ptr);
      }
      std::sort(
        entity_statuses.begin(), entity_statuses.end(),
        [](const auto & lhs, const auto & rhs) { return lhs.first < rhs.first; });

      const auto make_box_marker = [&](const std::string & name,
                                       const traffic_simulator::CanonicalizedEntityStatus & status,
                                       const int32_t marker_id) {
        const auto boost_polygon =
          math::geometry::toPolygon2D(status.getMapPose(), status.getBoundingBox());
        const auto & outer = boost_polygon.outer();
        if (outer.size() < 3) {
          return;
        }

        bool intersects_trajectory = false;
        if (!(canonicalized_entity_status && name == entity_name)) {
          for (const auto & trajectory_triangle : boost_polygons) {
            if (bg::intersects(trajectory_triangle, boost_polygon)) {
              intersects_trajectory = true;
              std::cout << entity_name << " npc trajectoryとnpc " << name << "は交点を持っている"
                        << std::endl;
              break;
            }
          }
        }

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = kFrameId;
        marker.header.stamp = stamp;
        marker.ns = ns + ":entity";
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = kOutlineWidth;

        std_msgs::msg::ColorRGBA color;
        color.a = 1.0F;
        if (canonicalized_entity_status && name == entity_name) {
          color.r = 0.0F;
          color.g = 1.0F;
          color.b = 0.0F;
        } else if (name == "ego" || name == "Ego") {
          color.r = 1.0F;
          color.g = 0.2F;
          color.b = 0.2F;
        } else {
          color.r = 1.0F;
          color.g = 1.0F;
          color.b = 0.0F;
        }
        marker.color = color;

        const double base_z = status.getMapPose().position.z + status.getBoundingBox().center.z;
        for (const auto & point : outer) {
          geometry_msgs::msg::Point geometry_point;
          geometry_point.x = point.x();
          geometry_point.y = point.y();
          geometry_point.z = base_z;
          marker.points.push_back(geometry_point);
        }
        if (!marker.points.empty()) {
          marker.points.push_back(marker.points.front());
          markers.emplace_back(std::move(marker));
        }

        if (intersects_trajectory) {
          visualization_msgs::msg::Marker text_marker;
          text_marker.header.frame_id = kFrameId;
          text_marker.header.stamp = stamp;
          text_marker.ns = ns + ":intersection_text";
          text_marker.id = marker_id;
          text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          text_marker.action = visualization_msgs::msg::Marker::ADD;
          text_marker.scale.z = 2.0;
          text_marker.color.r = 1.0F;
          text_marker.color.g = 0.0F;
          text_marker.color.b = 0.0F;
          text_marker.color.a = 1.0F;
          text_marker.pose.position = status.getMapPose().position;
          text_marker.pose.position.z += status.getBoundingBox().center.z +
                                         0.5 * status.getBoundingBox().dimensions.z + 1.0;
          text_marker.pose.orientation.x = 0.0;
          text_marker.pose.orientation.y = 0.0;
          text_marker.pose.orientation.z = 0.0;
          text_marker.pose.orientation.w = 1.0;
          text_marker.text = entity_name + " npc trajectoryとnpc " + name + "は交点を持っている";
          text_marker.lifetime = rclcpp::Duration::from_seconds(kLifetimeSeconds);
          markers.emplace_back(std::move(text_marker));
        }
      };

      const int32_t base_id = 10;
      for (std::size_t idx = 0; idx < entity_statuses.size(); ++idx) {
        if (idx > 0 && entity_statuses[idx].first == entity_statuses[idx - 1].first) {
          continue;
        }
        make_box_marker(
          entity_statuses[idx].first, *entity_statuses[idx].second,
          base_id + static_cast<int32_t>(idx));
      }
    }
  } catch (const std::exception & e) {
    std::cout << "[" << action_name << "] spline debug error: " << e.what() << std::endl;
  }
  const auto end = std::chrono::steady_clock::now();
  const auto elapsed_ms =
    std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start);
  std::cout << "[" << action_name << "] spline debug processing time: " << elapsed_ms.count()
            << " ms" << std::endl;
  publishImmediate(markers);
  return markers;
}
}  // namespace entity_behavior::vehicle::follow_lane_sequence
