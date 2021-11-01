// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <traffic_simulator/math/bounding_box.hpp>

// headers in Eigen
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <vector>

namespace traffic_simulator
{
namespace math
{
/**
 * @brief Get the Polygon Distance object
 *
 * @param pose0 pose of the first bounding box
 * @param bbox0 size of the first bounding box
 * @param pose1 pose of the second bounding box
 * @param bbox1 size of the second bounding box
 * @retval boost::none bounding box intersects
 * @retval 0 <= distance between two bounding boxes
 */
boost::optional<double> getPolygonDistance(
  const geometry_msgs::msg::Pose & pose0, const traffic_simulator_msgs::msg::BoundingBox & bbox0,
  const geometry_msgs::msg::Pose & pose1, const traffic_simulator_msgs::msg::BoundingBox & bbox1)
{
  const auto poly0 = get2DPolygon(pose0, bbox0);
  const auto poly1 = get2DPolygon(pose1, bbox1);
  if (boost::geometry::intersects(poly0, poly1)) {
    return boost::none;
  }
  if (boost::geometry::intersects(poly1, poly0)) {
    return boost::none;
  }
  if (boost::geometry::disjoint(poly0, poly1)) {
    return boost::geometry::distance(poly0, poly1);
  }
  return boost::none;
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
  traffic_simulator_msgs::msg::BoundingBox bbox)
{
  std::vector<geometry_msgs::msg::Point> points;
  geometry_msgs::msg::Point p0;
  p0.x = bbox.center.x + bbox.dimensions.x * 0.5;
  p0.y = bbox.center.y + bbox.dimensions.y * 0.5;
  p0.z = bbox.center.z + bbox.dimensions.z * 0.5;
  points.emplace_back(p0);
  geometry_msgs::msg::Point p1;
  p1.x = bbox.center.x - bbox.dimensions.x * 0.5;
  p1.y = bbox.center.y + bbox.dimensions.y * 0.5;
  p1.z = bbox.center.z + bbox.dimensions.z * 0.5;
  points.emplace_back(p1);
  geometry_msgs::msg::Point p2;
  p2.x = bbox.center.x - bbox.dimensions.x * 0.5;
  p2.y = bbox.center.y - bbox.dimensions.y * 0.5;
  p2.z = bbox.center.z + bbox.dimensions.z * 0.5;
  points.emplace_back(p2);
  geometry_msgs::msg::Point p3;
  p3.x = bbox.center.x + bbox.dimensions.x * 0.5;
  p3.y = bbox.center.y - bbox.dimensions.y * 0.5;
  p3.z = bbox.center.z + bbox.dimensions.z * 0.5;
  points.emplace_back(p3);
  return points;
}

std::vector<geometry_msgs::msg::Point> transformPoints(
  geometry_msgs::msg::Pose pose, std::vector<geometry_msgs::msg::Point> points)
{
  auto mat = quaternion_operation::getRotationMatrix(pose.orientation);
  std::vector<geometry_msgs::msg::Point> ret;
  for (const auto & point : points) {
    Eigen::VectorXd v(3);
    v(0) = point.x;
    v(1) = point.y;
    v(2) = point.z;
    v = mat * v;
    v(0) = v(0) + pose.position.x;
    v(1) = v(1) + pose.position.y;
    v(2) = v(2) + pose.position.z;
    geometry_msgs::msg::Point transformed;
    transformed.x = v(0);
    transformed.y = v(1);
    transformed.z = v(2);
    ret.emplace_back(transformed);
  }
  return ret;
}
}  // namespace math
}  // namespace traffic_simulator
