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

#include <awapi_awauto_adapter/awapi_simulation_api_publisher.hpp>

#include <string>

namespace autoware_api
{
AutowareSimulationAPIPublisher::AutowareSimulationAPIPublisher(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("awapi_traffic_simulator_publisher", options)
{
  // subscriber
  sub_initial_pose_ptr_ = create_subscription<PoseStamped>(
    "/initialpose",
    1,
    [&](const PoseStamped::SharedPtr msg_ptr)
    {
      initial_pose_ptr_ = msg_ptr;
    });

  sub_goal_pose_ptr_ = create_subscription<PoseStamped>(
    "/move_base_simple/goal",
    1,
    [&](const PoseStamped::SharedPtr msg_ptr)
    {
      goal_pose_ptr_ = msg_ptr;
    });
}
}  // namespace autoware_api
