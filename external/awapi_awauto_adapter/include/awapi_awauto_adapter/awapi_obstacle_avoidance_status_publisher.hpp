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

#ifndef AWAPI_AWAUTO_ADAPTER__AWAPI_OBSTACLE_AVOIDANCE_STATUS_PUBLISHER_HPP_
#define AWAPI_AWAUTO_ADAPTER__AWAPI_OBSTACLE_AVOIDANCE_STATUS_PUBLISHER_HPP_
#include <awapi_awauto_adapter/utility/visibility.h>

#include <autoware_api_msgs/msg/obstacle_avoidance_status.hpp>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

namespace autoware_api
{
class AutowareObstacleAvoidanceStatusPublisher : public rclcpp::Node
{
private:
  /**
   *  ObstacleAvoidanceStatus Topic: /awapi/obstacle_avoidance/get/status
   */
  using ObstacleAvoidanceStatus = autoware_api_msgs::msg::ObstacleAvoidanceStatus;
  rclcpp::Publisher<ObstacleAvoidanceStatus>::SharedPtr pub_obstacle_avoidance_status_;
  rclcpp::TimerBase::SharedPtr timer_obstacle_avoidance_status_;

public:
  AWAPI_AWAUTO_ADAPTER_PUBLIC
  /**
   * @brief publish obstacle avoidance
   * @return none
   */
  void publish_obstacle_avoidance_status();
  explicit AutowareObstacleAvoidanceStatusPublisher(const rclcpp::NodeOptions &);
};
}  // namespace autoware_api
#endif  // AWAPI_AWAUTO_ADAPTER__AWAPI_OBSTACLE_AVOIDANCE_STATUS_PUBLISHER_HPP_
