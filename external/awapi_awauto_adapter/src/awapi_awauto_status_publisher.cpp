// Copyright 2015-2020 TierIV.inc. All rights reserved.
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

#include <autoware_api_msgs/msg/awapi_autoware_status.hpp>
#include <awapi_awauto_adapter/awapi_awauto_status_publisher.hpp>

#include <string>

namespace autoware_api
{
AutowareAutoStatusPublisher::AutowareAutoStatusPublisher(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("awapi_awauto_state_pubrlisher", options)
{
  // publisher
  pub_autoware_status_ = this->create_publisher<AutowareStatus>("/awapi/autoware/get/status", 1);
}
void AutowareAutoStatusPublisher::publish_autoware_status()
{
  AutowareStatus autoware_status;
  autoware_status.header.frame_id = "base_link";
  autoware_status.header.stamp = get_clock()->now();
  autoware_status.autoware_state = get_autoware_state_info();
  autoware_status.control_mode = get_control_mode_info();
  autoware_status.gate_mode = get_gate_mode_info();
  autoware_status.emergency_stopped = get_emergency_info();
  RCLCPP_INFO(
    this->get_logger(), "AutowareStatus %i",
    autoware_status.header.stamp);
  pub_autoware_status_->publish(autoware_status);
}
std::string AutowareAutoStatusPublisher::get_autoware_state_info()
{
  std::string autoware_state = "test";
  return autoware_state;
}
int32_t AutowareAutoStatusPublisher::get_control_mode_info()
{
  int32_t control_mode = 3;
  return control_mode;
}

int32_t AutowareAutoStatusPublisher::get_gate_mode_info()
{
  int32_t gate_mode = 2;
  return gate_mode;
}

bool AutowareAutoStatusPublisher::get_emergency_info()
{
  std_msgs::msg::Bool is_emergency;
  is_emergency.data = true;
  return is_emergency.data;
}
}  // namespace autoware_api
