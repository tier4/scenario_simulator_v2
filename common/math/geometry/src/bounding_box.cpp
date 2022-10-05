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

#include <quaternion_operation/quaternion_operation.h>

#include <geometry/bounding_box.hpp>

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
  const auto poly0 = get2DPolygon(pose0, bbox0);
  const auto poly1 = get2DPolygon(pose1, bbox1);
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

const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> get2DPolygon(
  const geometry_msgs::msg::Pose & pose, const traffic_simulator_msgs::msg::BoundingBox & bbox)
{
  auto points = transformPoints(pose, getPointsFromBbox(bbox));
  typedef boost::geometry::model::d2::point_xy<double> bg_point;
  boost::geometry::model::polygon<bg_point> poly;
  poly.outer().push_back(bg_point(points[0].x, points[0].y));
  poly.outer().push_back(bg_point(points[1].x, points[1].y));
  poly.outer().push_back(bg_point(points[2].x, points[2].y));
  poly.outer().push_back(bg_point(points[3].x, points[3].y));
  poly.outer().push_back(bg_point(points[0].x, points[0].y));
  return poly;
}

std::vector<geometry_msgs::msg::Point> getPointsFromBbox(
  traffic_simulator_msgs::msg::BoundingBox bbox, double width_extension_right,
  double width_extension_left, double length_extension_front, double length_extension_rear)
{
  std::vector<geometry_msgs::msg::Point> points;
  geometry_msgs::msg::Point p0;
  p0.x = bbox.center.x + bbox.dimensions.x * 0.5 + length_extension_front;
  p0.y = bbox.center.y + bbox.dimensions.y * 0.5 + width_extension_left;
  p0.z = bbox.center.z + bbox.dimensions.z * 0.5;
  points.emplace_back(p0);
  geometry_msgs::msg::Point p1;
  p1.x = bbox.center.x - bbox.dimensions.x * 0.5 - length_extension_rear;
  p1.y = bbox.center.y + bbox.dimensions.y * 0.5 + width_extension_left;
  p1.z = bbox.center.z + bbox.dimensions.z * 0.5;
  points.emplace_back(p1);
  geometry_msgs::msg::Point p2;
  p2.x = bbox.center.x - bbox.dimensions.x * 0.5 - length_extension_rear;
  p2.y = bbox.center.y - bbox.dimensions.y * 0.5 - width_extension_right;
  p2.z = bbox.center.z + bbox.dimensions.z * 0.5;
  points.emplace_back(p2);
  geometry_msgs::msg::Point p3;
  p3.x = bbox.center.x + bbox.dimensions.x * 0.5 + length_extension_front;
  p3.y = bbox.center.y - bbox.dimensions.y * 0.5 - width_extension_right;
  p3.z = bbox.center.z + bbox.dimensions.z * 0.5;
  points.emplace_back(p3);
  return points;
}
}  // namespace geometry
}  // namespace math
