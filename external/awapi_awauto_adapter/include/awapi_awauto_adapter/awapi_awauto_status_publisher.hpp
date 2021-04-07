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

#ifndef AWAPI_AWAUTO_ADAPTER__AWAPI_AWAUTO_STATUS_PUBLISHER_HPP_
#define AWAPI_AWAUTO_ADAPTER__AWAPI_AWAUTO_STATUS_PUBLISHER_HPP_
#include <awapi_awauto_adapter/utility/visibility.h>

#include <autoware_api_msgs/msg/awapi_autoware_status.hpp>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace autoware_api
{
class AutowareAutoStatusPublisher : public rclcpp::Node
{
  /**
   * AutowareStatus Topic: /awapi/autoware/get/status
   */
  using AutowareStatus = autoware_api_msgs::msg::AwapiAutowareStatus;
  rclcpp::Publisher<AutowareStatus>::SharedPtr pub_autoware_status_;
  AutowareStatus autoware_status_;
  /**
   * @brief get autoware state info
   * @param none
   * @return int32_t
   * @todo make autoware state info
   */
  std::string get_autoware_state_info();
  /**
   * @brief get control mode info
   * @param none
   * @return int32_t
   * @todo make control mode info
   */
  int32_t get_control_mode_info();
  /**
   * @brief get gate mode info
   * @param none
   * @return int32_t
   * @todo make gate mode info
   */
  int32_t get_gate_mode_info();
  /**
   * @brief get emergency info
   * @param none
   * @return bool
   * @todo make emergency info
   */
  bool get_emergency_info();

public:
  AWAPI_AWAUTO_ADAPTER_PUBLIC
  explicit AutowareAutoStatusPublisher(const rclcpp::NodeOptions &);
  /**
   * @brief publish autoware status
   * @return none
   */
  void publish_autoware_status();
};
}  // namespace autoware_api
#endif  // AWAPI_AWAUTO_ADAPTER__AWAPI_AWAUTO_STATUS_PUBLISHER_HPP_
