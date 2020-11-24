// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include <awapi_awauto_adapter/awapi_obstacle_avoidance_status_publisher.hpp>

#include <string>
using ObstacleAvoidanceStatus = autoware_api_msgs::msg::ObstacleAvoidanceStatus;

namespace autoware_api
{
AutowareObstacleAvoidanceStatusPublisher::AutowareObstacleAvoidanceStatusPublisher(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("awapi_obstacle_avoidance_status_publisher", options)
{
  // publisher
  pub_obstacle_avoidance_status_ =
    this->create_publisher<ObstacleAvoidanceStatus>("/awapi/obstacle_avoidance/get/status", 1);
}
void AutowareObstacleAvoidanceStatusPublisher::publish_obstacle_avoidance_status()
{
  ObstacleAvoidanceStatus obstacle_avoidance_status;
  obstacle_avoidance_status.header.frame_id = "map";
  obstacle_avoidance_status.header.stamp = get_clock()->now();
  RCLCPP_INFO(
    this->get_logger(), "ObstacleAvoidanceStatus %i", obstacle_avoidance_status.header.stamp);
  pub_obstacle_avoidance_status_->publish(obstacle_avoidance_status);
}

}  // namespace autoware_api
