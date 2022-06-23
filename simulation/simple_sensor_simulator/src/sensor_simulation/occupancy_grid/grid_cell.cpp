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

#include <geometry/intersection/collision.hpp>
#include <geometry/polygon/polygon.hpp>
#include <rclcpp/rclcpp.hpp>
#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/grid.hpp>

namespace simple_sensor_simulator
{
GridCell::GridCell(
  const geometry_msgs::msg::Pose & origin, double size, size_t index, size_t row, size_t col)
: origin(origin), size(size), index(index), row(row), col(col), data_(0)
{
}

geometry_msgs::msg::Point GridCell::getLeftUpPoint() const
{
  geometry_msgs::msg::Point left_up;
  left_up.x = size * 0.5;
  left_up.y = size * 0.5;
  left_up = transformToWorld(left_up);
  return left_up;
}

geometry_msgs::msg::Point GridCell::getLeftDownPoint() const
{
  geometry_msgs::msg::Point left_down;
  left_down.x = size * 0.5;
  left_down.y = -size * 0.5;
  left_down = transformToWorld(left_down);
  return left_down;
}

geometry_msgs::msg::Point GridCell::getRightUpPoint() const
{
  geometry_msgs::msg::Point right_up;
  right_up.x = -size * 0.5;
  right_up.y = size * 0.5;
  right_up = transformToWorld(right_up);
  return right_up;
}

geometry_msgs::msg::Point GridCell::getRightDownPoint() const
{
  geometry_msgs::msg::Point right_down;
  right_down.x = -size * 0.5;
  right_down.y = -size * 0.5;
  right_down = transformToWorld(right_down);
  return right_down;
}

std::vector<geometry_msgs::msg::Point> GridCell::getPolygon() const
{
  return {getLeftUpPoint(), getLeftDownPoint(), getRightDownPoint(), getRightUpPoint()};
}

geometry_msgs::msg::Point GridCell::transformToWorld(const geometry_msgs::msg::Point & point) const
{
  auto mat = quaternion_operation::getRotationMatrix(origin.orientation);
  Eigen::VectorXd p(3);
  p(0) = point.x;
  p(1) = point.y;
  p(2) = point.z;
  p = mat * p;
  p(0) = p(0) + origin.position.x;
  p(1) = p(1) + origin.position.y;
  p(2) = p(2) + origin.position.z;
  geometry_msgs::msg::Point ret;
  ret.x = p(0);
  ret.y = p(1);
  ret.z = p(2);
  return ret;
}

bool GridCell::contains(const geometry_msgs::msg::Point & p) const
{
  return geometry_math::contains(getPolygon(), p);
}

bool GridCell::contains(const std::vector<geometry_msgs::msg::Point> & points) const
{
  for (const auto & point : points) {
    if (contains(point)) {
      return true;
    }
  }
  return false;
}

int8_t GridCell::getData() const { return data_; }
void GridCell::setData(int8_t data) { data_ = data; }
}  // namespace simple_sensor_simulator
