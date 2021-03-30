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

#include <simulation_api/math/bounding_box.hpp>

namespace simulation_api
{
namespace math
{
const std::vector<openscenario_msgs::msg::Point2D> get2DPolygon(
  const geometry_msgs::msg::Pose & pose,
  const openscenario_msgs::msg::BoundingBox & bbox)
{

}
std::vector<geometry_msgs::msg::Point> getPointsFromBbox(
  openscenario_msgs::msg::BoundingBox bbox)
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
  geometry_msgs::msg::Pose pose,
  std::vector<geometry_msgs::msg::Point> points)
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
}  // namespace simulation_api
