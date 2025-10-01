#include <behavior_tree_plugin/vehicle/follow_lane_sequence/spline_debug_calculator.hpp>

#include <geometry_msgs/msg/vector3.hpp>

#include <geometry/bounding_box.hpp>
#include <geometry/spline/catmull_rom_spline.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <cmath>
#include <utility>

// スプライン軌道を交差判定に使える幾何データへ変換する処理群の実装。

namespace entity_behavior::vehicle::follow_lane_sequence
{
// スプラインを等間隔で離散化し、幅情報から各区間の四角形とBGポリゴンを生成する。
auto buildQuadrilateralData(
  const math::geometry::CatmullRomSpline & spline, const double width,
  const std::size_t num_segments) -> QuadrilateralData
{
  QuadrilateralData data;
  if (num_segments == 0) {
    return data;
  }

  const double total_length = spline.getLength();
  const double step_size = num_segments > 0 ? total_length / static_cast<double>(num_segments) : 0.0;

  // 指定距離s地点におけるスプライン中心点から、左右境界点を求めるヘルパ。
  const auto computeBoundPoint = [&](const double s, const double direction) {
    geometry_msgs::msg::Vector3 normal_vector = spline.getNormalVector(s);
    const double theta = std::atan2(normal_vector.y, normal_vector.x);
    geometry_msgs::msg::Point center_point = spline.getPoint(s);
    geometry_msgs::msg::Point bound_point;
    bound_point.x = center_point.x + direction * 0.5 * width * std::cos(theta);
    bound_point.y = center_point.y + direction * 0.5 * width * std::sin(theta);
    bound_point.z = 1.0;
    return bound_point;
  };

  // 区間の端点ごとに左右境界座標を事前に確保しておく。
  std::vector<double> s_list;
  std::vector<geometry_msgs::msg::Point> left_bounds;
  std::vector<geometry_msgs::msg::Point> right_bounds;
  left_bounds.reserve(num_segments + 1);
  right_bounds.reserve(num_segments + 1);
  s_list.reserve(num_segments + 1);
  for (std::size_t i = 0; i <= num_segments; ++i) {
    const double s = step_size * static_cast<double>(i);
    right_bounds.emplace_back(computeBoundPoint(s, 1.0));
    left_bounds.emplace_back(computeBoundPoint(s, -1.0));
    s_list.emplace_back(s);
  }

  data.quadrilaterals.reserve(num_segments);
  data.polygons.reserve(num_segments);
  data.longitudinal_ranges.reserve(num_segments);
  for (std::size_t i = 0; i < num_segments; ++i) {
    // 連続する左右境界点4つで区間四角形を構成する。
    std::array<geometry_msgs::msg::Point, 4> quad{
      right_bounds[i], left_bounds[i], left_bounds[i + 1], right_bounds[i + 1]};
    data.quadrilaterals.emplace_back(quad);

    data.longitudinal_ranges.emplace_back(s_list[i], s_list[i + 1]);

    BoostPolygon polygon;
    auto & outer = polygon.outer();
    outer.reserve(5);
    // Boost.Geometryでは最初と最後の点が一致する必要があるため閉じる点を追加。
    outer.emplace_back(quad[0].x, quad[0].y);
    outer.emplace_back(quad[1].x, quad[1].y);
    outer.emplace_back(quad[2].x, quad[2].y);
    outer.emplace_back(quad[3].x, quad[3].y);
    outer.push_back(outer.front());
    bg::correct(polygon);
    data.polygons.emplace_back(polygon);
  }
  return data;
}

namespace
{
// 軌道ポリゴン集合と対象ポリゴンの交差有無を調べるユーティリティ。
auto intersectsTrajectory(
  const QuadrilateralData & trajectory_polygons, const BoostPolygon & target_polygon) -> std::pair<bool, double>
{
  const auto num = trajectory_polygons.polygons.size();
  for (std::size_t i = 0; i < num; ++i) {
    if (bg::intersects(trajectory_polygons.polygons.at(i), target_polygon)) {
      return std::make_pair(true, trajectory_polygons.longitudinal_ranges.at(i).first);
    }
  }
  return std::make_pair(false, -1.0);
}
}  // namespace

auto detectEntityCollisions(
  const QuadrilateralData & data,
  const std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus> &
    other_entity_status,
  const std::string & entity_name) -> std::vector<EntityCollisionInfo>
{
  std::vector<EntityCollisionInfo> collisions;
  if (data.polygons.empty()) {
    return collisions;
  }

  // 判定対象のエンティティ一覧（自車を除外）を作成する。
  std::vector<std::pair<std::string, const traffic_simulator::CanonicalizedEntityStatus *>> entity_statuses;
  entity_statuses.reserve(other_entity_status.size() + 1);
  for (const auto & [name, status] : other_entity_status) {
    if (entity_name == name) {
      continue;
    }
    entity_statuses.emplace_back(name, &status);
  }

  for (std::size_t idx = 0; idx < entity_statuses.size(); ++idx) {
    const auto * status = entity_statuses[idx].second;
    // エンティティの姿勢とバウンディングボックスから2Dポリゴンを生成。
    auto polygon = math::geometry::toPolygon2D(status->getMapPose(), status->getBoundingBox());
    if (polygon.outer().size() < 3) {
      continue;
    }

    // 軌道候補と交差するかを判定し、結果を蓄積する。
    const auto [intersects, range] = intersectsTrajectory(data, polygon);
    collisions.emplace_back(EntityCollisionInfo{
      entity_statuses[idx].first, status, std::move(polygon), intersects});
  }
  return collisions;
}
}  // namespace entity_behavior::vehicle::follow_lane_sequence
