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
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/spline_debug_calculator.hpp>

#include <geometry/spline/catmull_rom_spline.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <chrono>

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <iostream>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace entity_behavior::vehicle::follow_lane_sequence
{
namespace
{
// RViz 表示で使用する共通設定値。
constexpr char kFrameId[] = "map";
constexpr double kFillAlpha = 0.5;
constexpr double kLifetimeSeconds = 0.1;

// 区間四角形を TRIANGLE_LIST マーカーとして塗り潰し描画する。
auto makeQuadrilateralFillMarker(
  const std::string & ns, int32_t id, const rclcpp::Time & stamp,
  const std::vector<std::array<geometry_msgs::msg::Point, 4>> & quadrilaterals)
  -> visualization_msgs::msg::Marker
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = kFrameId;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.lifetime = rclcpp::Duration::from_seconds(kLifetimeSeconds);
  marker.color.r = 0.5F;
  marker.color.g = 0.5F;
  marker.color.b = 0.5F;
  marker.color.a = static_cast<float>(kFillAlpha);
  marker.points.reserve(quadrilaterals.size() * 6);

  for (std::size_t idx = 0; idx < quadrilaterals.size(); ++idx) {
    const auto & quad = quadrilaterals[idx];
    const auto add_triangle = [&](
                                   const geometry_msgs::msg::Point & a,
                                   const geometry_msgs::msg::Point & b,
                                   const geometry_msgs::msg::Point & c) {
      marker.points.push_back(a);
      marker.points.push_back(b);
      marker.points.push_back(c);
    };
    add_triangle(quad[0], quad[1], quad[2]);
    add_triangle(quad[0], quad[2], quad[3]);
  }

  return marker;
}

// 四角形群を RViz 表示用の塗り潰しマーカーへ変換する。
auto createSplineMarkers(
  const std::string & ns, const rclcpp::Time & stamp, const QuadrilateralData & data)
  -> std::vector<visualization_msgs::msg::Marker>
{
  std::vector<visualization_msgs::msg::Marker> markers;
  markers.reserve(1);
  markers.emplace_back(
    makeQuadrilateralFillMarker(ns + ":quadrilateral", 0, stamp, data.quadrilaterals));
  return markers;
}

struct ImmediateMarkerPublisher
{
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher;
};

// 即時可視化用ノード／パブリッシャを遅延初期化で取得する。
auto getImmediatePublisher() -> ImmediateMarkerPublisher &
{
  static ImmediateMarkerPublisher handles;
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);
  if (!handles.publisher) {
    try {
      // 即時可視化用の専用ノードとパブリッシャを遅延生成する。
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

// 即座に RViz へマーカー配列を送信するユーティリティ。
auto publishImmediate(const std::vector<visualization_msgs::msg::Marker> & markers) -> void
{
  auto & handles = getImmediatePublisher();
  if (!handles.publisher) {
    return;
  }
  // behavior_tree_plugin 側の出力とは別に、即時に RViz を更新する。
  visualization_msgs::msg::MarkerArray array_msg;
  array_msg.markers = markers;
  handles.publisher->publish(array_msg);
}
}  // namespace

// スプラインに基づくデバッグ情報を生成し、RViz に表示する。
void logSplineDebugInfo(
  const std::string & action_name,
  const traffic_simulator_msgs::msg::WaypointsArray & waypoints,
  const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> &
    canonicalized_entity_status,
  const std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus> &
    other_entity_status)
{
  // 処理時間を測定するために開始時刻を記録する。
  const auto start = std::chrono::steady_clock::now();
  static_cast<void>(other_entity_status);
  std::vector<visualization_msgs::msg::Marker> markers;
  const auto entity_name =
    canonicalized_entity_status ? canonicalized_entity_status->getName() : std::string("unknown");
  const auto ns = action_name + "_" + entity_name;
  const auto stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  try {
    math::geometry::CatmullRomSpline debug_spline(waypoints.waypoints);
    constexpr double kLaneWidth = 2.0;
    constexpr std::size_t kNumSegments = 50;
    const auto quadrilateral_data = buildQuadrilateralData(debug_spline, kLaneWidth, kNumSegments);

    auto spline_markers = createSplineMarkers(ns, stamp, quadrilateral_data);
    markers.insert(markers.end(), spline_markers.begin(), spline_markers.end());

  } catch (const std::exception & e) {
    std::cout << "[" << action_name << "] spline debug error: " << e.what() << std::endl;
  }
  const auto end = std::chrono::steady_clock::now();
  const auto elapsed_ms =
    std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start);
  // RViz publish を除いた処理時間をデバッグとして出力する。
  std::cout << "[" << action_name << "] spline debug processing time: " << elapsed_ms.count()
            << " ms" << std::endl;
  if (!markers.empty()) {
    publishImmediate(markers);
  }
}
}  // namespace entity_behavior::vehicle::follow_lane_sequence
