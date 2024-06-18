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

#include <tf2/LinearMath/Quaternion.h>

#include <geometry/quaternion/get_rotation.hpp>
#include <geometry/quaternion/get_rotation_matrix.hpp>
#include <geometry/transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace math
{
namespace geometry
{
const geometry_msgs::msg::Pose getRelativePose(
  const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to)
{
  geometry_msgs::msg::Transform from_translation;
  {
    from_translation.translation.x = from.position.x;
    from_translation.translation.y = from.position.y;
    from_translation.translation.z = from.position.z;
    from_translation.rotation = from.orientation;
  }

  tf2::Transform from_tf;
  {
    tf2::fromMsg(from_translation, from_tf);
  }

  geometry_msgs::msg::Transform to_translation;
  {
    to_translation.translation.x = to.position.x;
    to_translation.translation.y = to.position.y;
    to_translation.translation.z = to.position.z;
    to_translation.rotation = to.orientation;
  }

  tf2::Transform to_tf;
  {
    tf2::fromMsg(to_translation, to_tf);
  }

  tf2::Transform tf_delta = from_tf.inverse() * to_tf;

  geometry_msgs::msg::Pose ret;
  {
    tf2::toMsg(tf_delta, ret);
  }

  return ret;
}

const geometry_msgs::msg::Point transformPoint(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Point & point)
{
  auto mat = math::geometry::getRotationMatrix(pose.orientation);
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
  return transformed;
}

const geometry_msgs::msg::Point transformPoint(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Pose & sensor_pose,
  const geometry_msgs::msg::Point & point)
{
  auto mat = math::geometry::getRotationMatrix(
    math::geometry::getRotation(sensor_pose.orientation, pose.orientation));
  Eigen::VectorXd v(3);
  v(0) = point.x;
  v(1) = point.y;
  v(2) = point.z;
  v = mat * v;
  v(0) = v(0) + pose.position.x - sensor_pose.position.x;
  v(1) = v(1) + pose.position.y - sensor_pose.position.y;
  v(2) = v(2) + pose.position.z - sensor_pose.position.z;
  geometry_msgs::msg::Point ret;
  ret.x = v(0);
  ret.y = v(1);
  ret.z = v(2);
  return ret;
}

std::vector<geometry_msgs::msg::Point> transformPoints(
  const geometry_msgs::msg::Pose & pose, const std::vector<geometry_msgs::msg::Point> & points)
{
  std::vector<geometry_msgs::msg::Point> ret;
  std::transform(
    points.begin(), points.end(), std::back_inserter(ret),
    [pose](const geometry_msgs::msg::Point & point) { return transformPoint(pose, point); });
  return ret;
}

std::vector<geometry_msgs::msg::Point> transformPoints(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Pose & sensor_pose,
  const std::vector<geometry_msgs::msg::Point> & points)
{
  std::vector<geometry_msgs::msg::Point> ret;
  std::transform(
    points.begin(), points.end(), std::back_inserter(ret),
    [pose, sensor_pose](const geometry_msgs::msg::Point & point) {
      return transformPoint(pose, sensor_pose, point);
    });
  return ret;
}
}  // namespace geometry
}  // namespace math
