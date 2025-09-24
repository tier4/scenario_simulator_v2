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
#include <boost/geometry/algorithms/perimeter.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <chrono>

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <iostream>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace entity_behavior::vehicle::follow_lane_sequence
{
namespace
{
namespace bg = boost::geometry;
using BoostPoint = bg::model::d2::point_xy<double>;
using BoostPolygon = bg::model::polygon<BoostPoint>;
// RViz 表示で使用する共通設定値。
constexpr char kFrameId[] = "map";
constexpr double kFillAlpha = 0.4;
constexpr double kOutlineAlpha = 0.9;
constexpr double kOutlineWidth = 0.1;
constexpr double kLifetimeSeconds = 0.0;

struct QuadrilateralData
{
  std::vector<std::array<geometry_msgs::msg::Point, 4>> quadrilaterals;
  std::vector<BoostPolygon> polygons;
  double average_z{0.0};
};

// インデックスに応じて可視化用のベースカラーを決定する。
auto generateBaseColor(const std::size_t index) -> std::array<float, 3>
{
  constexpr std::array<std::array<float, 3>, 8> palette = {
    std::array<float, 3>{0.90F, 0.30F, 0.30F},
    std::array<float, 3>{0.30F, 0.80F, 0.35F},
    std::array<float, 3>{0.30F, 0.45F, 0.90F},
    std::array<float, 3>{0.90F, 0.80F, 0.30F},
    std::array<float, 3>{0.85F, 0.30F, 0.90F},
    std::array<float, 3>{0.30F, 0.90F, 0.90F},
    std::array<float, 3>{0.65F, 0.40F, 0.95F},
    std::array<float, 3>{0.95F, 0.55F, 0.35F}};
  return palette[index % palette.size()];
}

// 区間四角形を TRIANGLE_LIST マーカーとして塗り潰し描画する。
auto makeQuadrilateralFillMarker(
  const std::string & ns, int32_t id, const rclcpp::Time & stamp,
  const std::vector<std::array<geometry_msgs::msg::Point, 4>> & quadrilaterals,
  const std::vector<std_msgs::msg::ColorRGBA> & colors) -> visualization_msgs::msg::Marker
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
  marker.points.reserve(quadrilaterals.size() * 6);
  marker.colors.reserve(quadrilaterals.size() * 6);

  for (std::size_t idx = 0; idx < quadrilaterals.size(); ++idx) {
    const auto & quad = quadrilaterals[idx];
    const auto & color = colors[idx];
    const auto add_triangle = [&](
                                   const geometry_msgs::msg::Point & a,
                                   const geometry_msgs::msg::Point & b,
                                   const geometry_msgs::msg::Point & c) {
      marker.points.push_back(a);
      marker.colors.push_back(color);
      marker.points.push_back(b);
      marker.colors.push_back(color);
      marker.points.push_back(c);
      marker.colors.push_back(color);
    };
    add_triangle(quad[0], quad[1], quad[2]);
    add_triangle(quad[0], quad[2], quad[3]);
  }

  return marker;
}

// 区間四角形の輪郭線を LINE_LIST マーカーとして生成する。
auto makeQuadrilateralOutlineMarker(
  const std::string & ns, int32_t id, const rclcpp::Time & stamp,
  const std::vector<std::array<geometry_msgs::msg::Point, 4>> & quadrilaterals,
  const std::vector<std_msgs::msg::ColorRGBA> & colors) -> visualization_msgs::msg::Marker
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = kFrameId;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = kOutlineWidth;
  marker.lifetime = rclcpp::Duration::from_seconds(kLifetimeSeconds);

  for (std::size_t idx = 0; idx < quadrilaterals.size(); ++idx) {
    const auto & quad = quadrilaterals[idx];
    const auto & color = colors[idx];
    for (std::size_t vertex = 0; vertex < quad.size(); ++vertex) {
      const auto & from = quad[vertex];
      const auto & to = quad[(vertex + 1) % quad.size()];
      marker.points.push_back(from);
      marker.colors.push_back(color);
      marker.points.push_back(to);
      marker.colors.push_back(color);
    }
  }

  return marker;
}

// 一度に全ての既存マーカーを無効化する DELETEALL マーカーを作成する。
auto makeDeleteAllMarker(const rclcpp::Time & stamp) -> visualization_msgs::msg::Marker
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = kFrameId;
  marker.header.stamp = stamp;
  marker.ns = "spline_debug";
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

// Catmull-Rom 曲線を等分し、各区間を四角形に展開して判定用ポリゴンを構築する。
auto buildQuadrilateralData(
  const math::geometry::CatmullRomSpline & spline, const double width,
  const std::size_t num_segments, const double z_offset) -> QuadrilateralData
{
  QuadrilateralData data;
  if (num_segments == 0) {
    return data;
  }

  const double total_length = spline.getLength();
  const double step_size = num_segments > 0 ? total_length / static_cast<double>(num_segments) : 0.0;

  const auto computeBoundPoint = [&](const double s, const double direction) {
    geometry_msgs::msg::Vector3 normal_vector = spline.getNormalVector(s);
    const double theta = std::atan2(normal_vector.y, normal_vector.x);
    geometry_msgs::msg::Point center_point = spline.getPoint(s);
    geometry_msgs::msg::Point bound_point;
    bound_point.x = center_point.x + direction * 0.5 * width * std::cos(theta);
    bound_point.y = center_point.y + direction * 0.5 * width * std::sin(theta);
    bound_point.z = center_point.z + z_offset;
    return bound_point;
  };

  std::vector<geometry_msgs::msg::Point> left_bounds;
  std::vector<geometry_msgs::msg::Point> right_bounds;
  left_bounds.reserve(num_segments + 1);
  right_bounds.reserve(num_segments + 1);
  for (std::size_t i = 0; i <= num_segments; ++i) {
    const double s = step_size * static_cast<double>(i);
    right_bounds.emplace_back(computeBoundPoint(s, 1.0));
    left_bounds.emplace_back(computeBoundPoint(s, -1.0));
  }

  double accumulated_z = 0.0;
  data.quadrilaterals.reserve(num_segments);
  data.polygons.reserve(num_segments);
  for (std::size_t i = 0; i < num_segments; ++i) {
    std::array<geometry_msgs::msg::Point, 4> quad{
      right_bounds[i], left_bounds[i], left_bounds[i + 1], right_bounds[i + 1]};
    for (const auto & point : quad) {
      accumulated_z += point.z;
    }
    data.quadrilaterals.emplace_back(quad);

    BoostPolygon polygon;
    auto & outer = polygon.outer();
    outer.reserve(5);
    outer.emplace_back(quad[0].x, quad[0].y);
    outer.emplace_back(quad[1].x, quad[1].y);
    outer.emplace_back(quad[2].x, quad[2].y);
    outer.emplace_back(quad[3].x, quad[3].y);
    outer.push_back(outer.front());
    bg::correct(polygon);
    data.polygons.emplace_back(polygon);
  }

  if (!data.quadrilaterals.empty()) {
    data.average_z = accumulated_z / (static_cast<double>(data.quadrilaterals.size()) * 4.0);
  }
  return data;
}

// 四角形群を RViz 表示用の塗り潰し／輪郭マーカーへ変換する。
auto createSplineMarkers(
  const std::string & ns, const rclcpp::Time & stamp, const QuadrilateralData & data)
  -> std::vector<visualization_msgs::msg::Marker>
{
  std::vector<std_msgs::msg::ColorRGBA> fill_colors;
  fill_colors.reserve(data.quadrilaterals.size());
  for (std::size_t idx = 0; idx < data.quadrilaterals.size(); ++idx) {
    const auto base_color = generateBaseColor(idx);
    std_msgs::msg::ColorRGBA color;
    color.r = base_color[0];
    color.g = base_color[1];
    color.b = base_color[2];
    color.a = static_cast<float>(kFillAlpha);
    fill_colors.emplace_back(color);
  }
  auto outline_colors = fill_colors;
  for (auto & color : outline_colors) {
    color.a = static_cast<float>(kOutlineAlpha);
  }

  std::vector<visualization_msgs::msg::Marker> markers;
  markers.reserve(2);
  markers.emplace_back(makeQuadrilateralFillMarker(
    ns + ":quadrilateral_fill", 0, stamp, data.quadrilaterals, fill_colors));
  markers.emplace_back(makeQuadrilateralOutlineMarker(
    ns + ":quadrilateral_outline", 0, stamp, data.quadrilaterals, outline_colors));
  return markers;
}

// 生成済み軌道ポリゴンと対象ポリゴンの交差有無を調べる。
auto intersectsTrajectory(
  const std::vector<BoostPolygon> & trajectory_polygons, const BoostPolygon & target_polygon) -> bool
{
  for (const auto & trajectory_polygon : trajectory_polygons) {
    if (bg::intersects(trajectory_polygon, target_polygon)) {
      return true;
    }
  }
  return false;
}

// 軌道と交差する可能性のあるエンティティの枠と警告テキストを生成する。
auto createEntityMarkers(
  const std::string & ns, const rclcpp::Time & stamp,
  const std::vector<BoostPolygon> & trajectory_polygons,
  const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> & canonicalized_entity_status,
  const std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus> & other_entity_status,
  const std::string & entity_name) -> std::vector<visualization_msgs::msg::Marker>
{
  std::vector<visualization_msgs::msg::Marker> markers;
  if (trajectory_polygons.empty()) {
    return markers;
  }

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

  const int32_t base_id = 10;
  for (std::size_t idx = 0; idx < entity_statuses.size(); ++idx) {
    if (idx > 0 && entity_statuses[idx].first == entity_statuses[idx - 1].first) {
      continue;
    }

    const auto & name = entity_statuses[idx].first;
    const auto & status = *entity_statuses[idx].second;
    const auto boost_polygon = math::geometry::toPolygon2D(status.getMapPose(), status.getBoundingBox());
    const auto & outer = boost_polygon.outer();
    if (outer.size() < 3) {
      continue;
    }

    bool intersects_trajectory = false;
    if (!(canonicalized_entity_status && name == entity_name)) {
      intersects_trajectory = intersectsTrajectory(trajectory_polygons, boost_polygon);
      if (intersects_trajectory) {
        std::cout << entity_name << " npc trajectoryとnpc " << name << "は交点を持っている" << std::endl;
      }
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = kFrameId;
    marker.header.stamp = stamp;
    marker.ns = ns + ":entity";
    marker.id = base_id + static_cast<int32_t>(idx);
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
      text_marker.id = base_id + static_cast<int32_t>(idx);
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
  }

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
auto logSplineDebugInfo(
  const std::string & action_name,
  const traffic_simulator_msgs::msg::WaypointsArray & waypoints,
  const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> &
    canonicalized_entity_status,
  const std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus> &
    other_entity_status) -> std::vector<visualization_msgs::msg::Marker>
{
  // 処理時間を測定するために開始時刻を記録する。
  const auto start = std::chrono::steady_clock::now();
  std::vector<visualization_msgs::msg::Marker> markers;
  const auto entity_name =
    canonicalized_entity_status ? canonicalized_entity_status->getName() : std::string("unknown");
  const auto ns = action_name + "_" + entity_name;
  const auto stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  try {
    math::geometry::CatmullRomSpline debug_spline(waypoints.waypoints);
    constexpr double kLaneWidth = 1.0;
    constexpr std::size_t kNumSegments = 30;
    constexpr double kZOffset = 0.0;
    const auto quadrilateral_data = buildQuadrilateralData(debug_spline, kLaneWidth, kNumSegments, kZOffset);

    std::cout << "[" << action_name << "] [" << entity_name << "] quadrilateral count="
              << quadrilateral_data.quadrilaterals.size();
    if (!quadrilateral_data.quadrilaterals.empty()) {
      const auto & first_point = quadrilateral_data.quadrilaterals.front().front();
      const auto & last_point = quadrilateral_data.quadrilaterals.back().back();
      std::cout << " first=(" << first_point.x << ", " << first_point.y << ", " << first_point.z
                << ")";
      std::cout << " last=(" << last_point.x << ", " << last_point.y << ", " << last_point.z
                << ")";
      std::cout << " average_z=" << quadrilateral_data.average_z;
    }
    double trajectory_outer_perimeter = 0.0;
    for (const auto & polygon : quadrilateral_data.polygons) {
      trajectory_outer_perimeter += bg::perimeter(polygon.outer());
    }
    std::cout << " trajectory outer perimeter=" << trajectory_outer_perimeter << std::endl;

    auto spline_markers = createSplineMarkers(ns, stamp, quadrilateral_data);
    markers.insert(markers.end(), spline_markers.begin(), spline_markers.end());

    auto entity_markers = createEntityMarkers(
      ns, stamp, quadrilateral_data.polygons, canonicalized_entity_status, other_entity_status,
      entity_name);
    markers.insert(markers.end(), entity_markers.begin(), entity_markers.end());

  } catch (const std::exception & e) {
    std::cout << "[" << action_name << "] spline debug error: " << e.what() << std::endl;
  }
  const auto end = std::chrono::steady_clock::now();
  const auto elapsed_ms =
    std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start);
  // RViz publish を除いた処理時間をデバッグとして出力する。
  std::cout << "[" << action_name << "] spline debug processing time: " << elapsed_ms.count()
            << " ms" << std::endl;
  const auto delete_all_marker = makeDeleteAllMarker(stamp);
  publishImmediate(std::vector<visualization_msgs::msg::Marker>{delete_all_marker});
  if (!markers.empty()) {
    publishImmediate(markers);
  }
  std::vector<visualization_msgs::msg::Marker> returned_markers;
  returned_markers.reserve(markers.size() + 1);
  returned_markers.emplace_back(delete_all_marker);
  returned_markers.insert(returned_markers.end(), markers.begin(), markers.end());
  return returned_markers;
}
}  // namespace entity_behavior::vehicle::follow_lane_sequence
