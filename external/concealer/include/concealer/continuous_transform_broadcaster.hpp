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

#ifndef CONCEALER__CONTINUOUS_TRANSFORM_BROADCASTER_HPP_
#define CONCEALER__CONTINUOUS_TRANSFORM_BROADCASTER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace concealer
{
template <typename Node>
class ContinuousTransformBroadcaster
{
  const std::string child_frame_id;

  tf2_ros::Buffer transform_buffer;

  tf2_ros::TransformBroadcaster transform_broadcaster;

  geometry_msgs::msg::TransformStamped current_transform;

  const rclcpp::TimerBase::SharedPtr timer;

  void updateTransform()
  {
    if (
      not current_transform.header.frame_id.empty() and
      not current_transform.child_frame_id.empty())  //
    {
      current_transform.header.stamp = static_cast<Node &>(*this).get_clock()->now();
      return transform_broadcaster.sendTransform(current_transform);
    }
  }

public:
  const auto & setTransform(const geometry_msgs::msg::Pose & pose)
  {
    current_transform.header.stamp = static_cast<Node &>(*this).get_clock()->now();
    current_transform.header.frame_id = "map";
    current_transform.child_frame_id = child_frame_id;
    current_transform.transform.translation.x = pose.position.x;
    current_transform.transform.translation.y = pose.position.y;
    current_transform.transform.translation.z = pose.position.z;
    current_transform.transform.rotation = pose.orientation;

    return current_transform;
  }

  explicit ContinuousTransformBroadcaster(const std::string & child_frame_id)
  : child_frame_id(child_frame_id),
    transform_buffer(static_cast<Node &>(*this).get_clock()),
    transform_broadcaster(static_cast<Node *>(this)),
    timer(static_cast<Node &>(*this).create_wall_timer(
      std::chrono::milliseconds(5), [this]() { return updateTransform(); }))
  {
  }
};
}  // namespace concealer

#endif  // CONCEALER__CONTINUOUS_TRANSFORM_BROADCASTER_HPP_
