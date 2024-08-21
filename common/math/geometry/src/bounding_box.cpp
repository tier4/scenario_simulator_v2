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

#include <geometry/bounding_box.hpp>
#include <geometry/polygon/polygon.hpp>

// headers in Eigen
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <optional>
#include <vector>

namespace math
{
namespace geometry
{
/**
 * @brief Get the Polygon Distance object
 *
 * @param pose0 pose of the first bounding box
 * @param bbox0 size of the first bounding box
 * @param pose1 pose of the second bounding box
 * @param bbox1 size of the second bounding box
 * @retval std::nullopt bounding box intersects
 * @retval 0 <= distance between two bounding boxes
 */
std::optional<double> getPolygonDistance(
  const geometry_msgs::msg::Pose & pose0, const traffic_simulator_msgs::msg::BoundingBox & bbox0,
  const geometry_msgs::msg::Pose & pose1, const traffic_simulator_msgs::msg::BoundingBox & bbox1)
{
  const auto poly0 = toPolygon2D(pose0, bbox0);
  const auto poly1 = toPolygon2D(pose1, bbox1);
  if (boost::geometry::intersects(poly0, poly1)) {
    return std::nullopt;
  }
  if (boost::geometry::intersects(poly1, poly0)) {
    return std::nullopt;
  }
  if (boost::geometry::disjoint(poly0, poly1)) {
    return boost::geometry::distance(poly0, poly1);
  }
  return std::nullopt;
}

// inspiration taken from
// https://stackoverflow.com/questions/62138176/how-to-find-two-points-that-form-closest-distance-between-two-rectangles
std::optional<std::pair<geometry_msgs::msg::Pose, geometry_msgs::msg::Pose>> getClosestPoses(
  const geometry_msgs::msg::Pose & pose0, const traffic_simulator_msgs::msg::BoundingBox & bbox0,
  const geometry_msgs::msg::Pose & pose1, const traffic_simulator_msgs::msg::BoundingBox & bbox1)
{
  const auto poly0 = toPolygon2D(pose0, bbox0);
  const auto poly1 = toPolygon2D(pose1, bbox1);

  if (boost::geometry::intersects(poly0, poly1)) {
    return std::nullopt;
  }
  if (boost::geometry::intersects(poly1, poly0)) {
    return std::nullopt;
  }
  if (boost::geometry::disjoint(poly0, poly1)) {
    auto point0 = boost_point();
    auto point1 = boost_point();
    auto min_distance = boost::numeric::bounds<double>::highest();

    auto segments = boost::make_iterator_range(
      boost::geometry::segments_begin(poly0), boost::geometry::segments_end(poly0));
    auto points = boost::make_iterator_range(
      boost::geometry::points_begin(poly1), boost::geometry::points_end(poly1));
    for (auto && segment : segments) {
      for (auto && point : points) {
        auto nearest_point_from_segment =
          pointToSegmentProjection(point, *segment.first, *segment.second);
        auto distance = boost::geometry::distance(point, nearest_point_from_segment);
        if (distance < min_distance) {
          min_distance = distance;
          point0 = point;
          point1 = nearest_point_from_segment;
        }
      }
    }

    return std::make_pair(toPose(point0), toPose(point1));
  }
  return std::nullopt;
}

boost_point pointToSegmentProjection(
  const boost_point & p, const boost_point & p1, const boost_point & p2)
{
  boost_point v = p2, w = p;
  boost::geometry::subtract_point(v, p1);
  boost::geometry::subtract_point(w, p1);

  auto const c1 = boost::geometry::dot_product(w, v);
  if (c1 <= 0) return p1;

  auto const c2 = boost::geometry::dot_product(v, v);
  if (c2 <= c1) return p2;

  boost_point prj = p1;
  boost::geometry::multiply_value(v, c1 / c2);
  boost::geometry::add_point(prj, v);

  return prj;
}

auto toPolygon2D(
  const geometry_msgs::msg::Pose & pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box) -> const boost_polygon
{
  return toBoostPolygon(transformPoints(pose, getPointsFromBbox(bounding_box)));
}

std::vector<geometry_msgs::msg::Point> getPointsFromBbox(
  traffic_simulator_msgs::msg::BoundingBox bounding_box, double width_extension_right,
  double width_extension_left, double length_extension_front, double length_extension_rear)
{
  std::vector<geometry_msgs::msg::Point> points;
  auto distances_from_center_to_edge = getDistancesFromCenterToEdge(bounding_box);
  geometry_msgs::msg::Point p0;
  p0.x = distances_from_center_to_edge.front + length_extension_front;
  p0.y = distances_from_center_to_edge.left + width_extension_left;
  p0.z = distances_from_center_to_edge.up;
  points.emplace_back(p0);
  geometry_msgs::msg::Point p1;
  p1.x = distances_from_center_to_edge.rear - length_extension_rear;
  p1.y = distances_from_center_to_edge.left + width_extension_left;
  p1.z = distances_from_center_to_edge.up;
  points.emplace_back(p1);
  geometry_msgs::msg::Point p2;
  p2.x = distances_from_center_to_edge.rear - length_extension_rear;
  p2.y = distances_from_center_to_edge.right - width_extension_right;
  p2.z = distances_from_center_to_edge.up;
  points.emplace_back(p2);
  geometry_msgs::msg::Point p3;
  p3.x = distances_from_center_to_edge.front + length_extension_front;
  p3.y = distances_from_center_to_edge.right - width_extension_right;
  p3.z = distances_from_center_to_edge.up;
  points.emplace_back(p3);
  return points;
}

auto toPolygon2D(const traffic_simulator_msgs::msg::BoundingBox & bounding_box)
  -> std::vector<geometry_msgs::msg::Point>
{
  std::vector<geometry_msgs::msg::Point> points_bounding_box;
  geometry_msgs::msg::Point p0, p1, p2, p3, p4, p5, p6, p7;

  p0.x = bounding_box.center.x + bounding_box.dimensions.x * 0.5;
  p0.y = bounding_box.center.y + bounding_box.dimensions.y * 0.5;
  p0.z = bounding_box.center.z + bounding_box.dimensions.z * 0.5;
  points_bounding_box.emplace_back(p0);

  p1.x = bounding_box.center.x + bounding_box.dimensions.x * 0.5;
  p1.y = bounding_box.center.y + bounding_box.dimensions.y * 0.5;
  p1.z = bounding_box.center.z - bounding_box.dimensions.z * 0.5;
  points_bounding_box.emplace_back(p1);

  p2.x = bounding_box.center.x + bounding_box.dimensions.x * 0.5;
  p2.y = bounding_box.center.y - bounding_box.dimensions.y * 0.5;
  p2.z = bounding_box.center.z + bounding_box.dimensions.z * 0.5;
  points_bounding_box.emplace_back(p2);

  p3.x = bounding_box.center.x - bounding_box.dimensions.x * 0.5;
  p3.y = bounding_box.center.y + bounding_box.dimensions.y * 0.5;
  p3.z = bounding_box.center.z + bounding_box.dimensions.z * 0.5;
  points_bounding_box.emplace_back(p3);

  p4.x = bounding_box.center.x + bounding_box.dimensions.x * 0.5;
  p4.y = bounding_box.center.y - bounding_box.dimensions.y * 0.5;
  p4.z = bounding_box.center.z - bounding_box.dimensions.z * 0.5;
  points_bounding_box.emplace_back(p4);

  p5.x = bounding_box.center.x - bounding_box.dimensions.x * 0.5;
  p5.y = bounding_box.center.y + bounding_box.dimensions.y * 0.5;
  p5.z = bounding_box.center.z - bounding_box.dimensions.z * 0.5;
  points_bounding_box.emplace_back(p5);

  p6.x = bounding_box.center.x - bounding_box.dimensions.x * 0.5;
  p6.y = bounding_box.center.y - bounding_box.dimensions.y * 0.5;
  p6.z = bounding_box.center.z + bounding_box.dimensions.z * 0.5;
  points_bounding_box.emplace_back(p6);

  p7.x = bounding_box.center.x - bounding_box.dimensions.x * 0.5;
  p7.y = bounding_box.center.y - bounding_box.dimensions.y * 0.5;
  p7.z = bounding_box.center.z - bounding_box.dimensions.z * 0.5;
  points_bounding_box.emplace_back(p7);

  return math::geometry::get2DConvexHull(points_bounding_box);
}

boost_point toBoostPoint(const geometry_msgs::msg::Point & point)
{
  return boost_point(point.x, point.y);
}

boost_polygon toBoostPolygon(const std::vector<geometry_msgs::msg::Point> & points)
{
  boost_polygon poly;
  poly.outer().push_back(toBoostPoint(points[0]));
  poly.outer().push_back(toBoostPoint(points[1]));
  poly.outer().push_back(toBoostPoint(points[2]));
  poly.outer().push_back(toBoostPoint(points[3]));
  poly.outer().push_back(toBoostPoint(points[0]));
  return poly;
}

geometry_msgs::msg::Pose toPose(const boost_point & point)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = point.x();
  pose.position.y = point.y();

  return pose;
}

geometry_msgs::msg::Pose subtractPoses(
  const geometry_msgs::msg::Pose & pose1, const geometry_msgs::msg::Pose & pose2)
{
  auto point1 = toBoostPoint(pose1.position);
  auto point2 = toBoostPoint(pose2.position);

  boost::geometry::subtract_point(point1, point2);

  return toPose(point1);
}

DistancesFromCenterToEdge getDistancesFromCenterToEdge(
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box)
{
  DistancesFromCenterToEdge distances;
  distances.front = bounding_box.center.x + bounding_box.dimensions.x * 0.5;
  distances.rear = bounding_box.center.x - bounding_box.dimensions.x * 0.5;
  distances.left = bounding_box.center.y + bounding_box.dimensions.y * 0.5;
  distances.right = bounding_box.center.y - bounding_box.dimensions.y * 0.5;
  distances.up = bounding_box.center.z + bounding_box.dimensions.z * 0.5;
  distances.down = bounding_box.center.z - bounding_box.dimensions.z * 0.5;

  return distances;
}

}  // namespace geometry
}  // namespace math
