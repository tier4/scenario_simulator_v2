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

#ifndef AWAPI_AWAUTO_ADAPTER__AWAPI_SIMULATION_API_PUBLISHER_HPP_
#define AWAPI_AWAUTO_ADAPTER__AWAPI_SIMULATION_API_PUBLISHER_HPP_
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <chrono>
#include <memory>

namespace autoware_api
{
class AutowareSimulationAPIPublisher : public rclcpp::Node
{
private:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  // subscriber
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_initial_pose_ptr_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_goal_pose_ptr_;
  // publisher
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_initial_pose_ptr_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_goal_pose_ptr_;
  PoseStamped::SharedPtr initial_pose_ptr_;
  PoseStamped::SharedPtr goal_pose_ptr_;
  /**
   * @brief send initial pose
   * @return none
   */
  void send_initail_pose();
  /**
   * @brief send goal pose
   * @return none
   */
  void send_goal_pose();

public:
  explicit AutowareSimulationAPIPublisher(const rclcpp::NodeOptions &);
};
}  // namespace autoware_api
#endif  // AWAPI_AWAUTO_ADAPTER__AWAPI_SIMULATION_API_PUBLISHER_HPP_
