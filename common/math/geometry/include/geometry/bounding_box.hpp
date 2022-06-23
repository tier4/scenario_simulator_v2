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

#ifndef GEOMETRY__BOUNDING_BOX_HPP_
#define GEOMETRY__BOUNDING_BOX_HPP_

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/optional.hpp>
#include <geometry/transform.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <vector>

namespace math
{
namespace geometry
{
boost::optional<double> getPolygonDistance(
  const geometry_msgs::msg::Pose & pose0, const traffic_simulator_msgs::msg::BoundingBox & bbox0,
  const geometry_msgs::msg::Pose & pose1, const traffic_simulator_msgs::msg::BoundingBox & bbox1);
const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> get2DPolygon(
  const geometry_msgs::msg::Pose & pose, const traffic_simulator_msgs::msg::BoundingBox & bbox);
std::vector<geometry_msgs::msg::Point> getPointsFromBbox(
  traffic_simulator_msgs::msg::BoundingBox bbox, double width_extension_right = 0.0,
  double width_extension_left = 0.0, double length_extension_front = 0.0,
  double length_extension_rear = 0.0);
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__BOUNDING_BOX_HPP_
