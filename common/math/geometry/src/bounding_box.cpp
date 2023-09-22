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
#include <iostream>
#include <fstream>

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

std::optional<double> getPolygonDistance(
  const geometry_msgs::msg::Pose & pose0, const traffic_simulator_msgs::msg::BoundingBox & bbox0,
  const geometry_msgs::msg::Pose & pose1)
{
  const auto poly0 = get2DPolygon(pose0, bbox0);
  const auto point1 = boost_point(pose1.position.x, pose1.position.y);
  if (boost::geometry::intersects(poly0, point1)) {
    return std::nullopt;
  }
  if (boost::geometry::intersects(point1, poly0)) {
    return std::nullopt;
  }
  if (boost::geometry::disjoint(poly0, point1)) {
    return boost::geometry::distance(poly0, point1);
  }
  return std::nullopt;
}

// inspiration taken from
// https://stackoverflow.com/questions/62138176/how-to-find-two-points-that-form-closest-distance-between-two-rectangles
std::optional<geometry_msgs::msg::Pose> getClosestPose(
  const geometry_msgs::msg::Pose & pose0, const traffic_simulator_msgs::msg::BoundingBox & bbox0,
  const geometry_msgs::msg::Pose & pose1)
{
  const auto poly0 = get2DPolygon(pose0, bbox0);
  const auto point1 = toBoostPoint(pose1.position);

  if (boost::geometry::intersects(poly0, point1)) {
    return std::nullopt;
  }
  if (boost::geometry::intersects(point1, poly0)) {
    return std::nullopt;
  }
  if (boost::geometry::disjoint(poly0, point1)) {
// RCLCPP_INFO_STREAM(rclcpp::get_logger("relative pose"),  "s-----------------------------------------------------------");
// RCLCPP_INFO_STREAM(rclcpp::get_logger("relative pose"), "pose0 " << pose0.position.x << " " << pose0.position.y << "");
// RCLCPP_INFO_STREAM(rclcpp::get_logger("relative pose"), "pose1 " << pose1.position.x << " " << pose1.position.y << "");

// RCLCPP_INFO_STREAM(rclcpp::get_logger("relative pose"), "poly0 " << boost::geometry::wkt(poly0));
// RCLCPP_INFO_STREAM(rclcpp::get_logger("relative pose"), "point1 " << boost::geometry::wkt(point1));
// RCLCPP_INFO_STREAM(rclcpp::get_logger("relative pose"), "dist " << boost::geometry::distance(poly0, point1));

    auto nearest = boost_point();
    auto min_d = boost::numeric::bounds<double>::highest();
    auto segments = boost::make_iterator_range(boost::geometry::segments_begin(poly0), boost::geometry::segments_end(poly0));
    for (auto&& pq : segments) {
          auto nearest_point_from_segment = point_to_segment(point1, *pq.first, *pq.second);
          auto cmpDst = boost::geometry::distance(nearest_point_from_segment,point1);
            if (cmpDst < min_d) {
                min_d = cmpDst;
                nearest = nearest_point_from_segment;
            }
    }

    // point_type min_p;

// RCLCPP_INFO_STREAM(rclcpp::get_logger("relative pose"), "dist " << boost::geometry::distance(nearest, point1));
    // auto p = geometry_msgs::msg::Pose();
    // p.position.x = nearest.x();
    // p.position.y = nearest.y();
// RCLCPP_INFO_STREAM(rclcpp::get_logger("relative pose"), "f-----------------------------------------------------------");
    return toPose(nearest);
  }
  return std::nullopt;
}

boost_point point_to_segment(boost_point const& p, boost_point const& p1, boost_point const& p2)
{
    boost_point v = p2, w = p;
    boost::geometry::subtract_point(v, p1);
    boost::geometry::subtract_point(w, p1);

    auto const c1 = boost::geometry::dot_product(w, v);
    if (c1 <= 0)  return p1;

    auto const c2 = boost::geometry::dot_product(v, v);
    if (c2 <= c1) return p2;

    boost_point prj = p1;
    boost::geometry::multiply_value(v, c1/c2);
    boost::geometry::add_point(prj, v);

    return prj;
}

const boost_polygon get2DPolygon(
  const geometry_msgs::msg::Pose & pose, const traffic_simulator_msgs::msg::BoundingBox & bbox)
{
  return toBoostPoly(transformPoints(pose, getPointsFromBbox(bbox)));
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

boost_point toBoostPoint(const geometry_msgs::msg::Point & point)
{
  return boost_point(point.x, point.y);
}

boost_polygon toBoostPoly(
  const std::vector<geometry_msgs::msg::Point> & points)
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

geometry_msgs::msg::Pose subtractPoses(const geometry_msgs::msg::Pose & pose1, const geometry_msgs::msg::Pose & pose2)
{
  auto v = toBoostPoint(pose1.position);
  auto w = toBoostPoint(pose2.position);

  boost::geometry::subtract_point(v, w);

  return toPose(v);
}

}  // namespace geometry
}  // namespace math
