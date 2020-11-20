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
#include <autoware_api_msgs/msg/awapi_autoware_status.hpp>
#include <awapi_awauto_adapter/awapi_awauto_status_publisher.hpp>

#include <string>

namespace autoware_api
{
AutowareAutoStatusPublisher::AutowareAutoStatusPublisher(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("awapi_awauto_state_publisher", options)
{
  // publisher
  pub_autoware_status_ = this->create_publisher<AutowareStatus>("/awapi/autoware/get/status", 1);
}

void AutowareAutoStatusPublisher::publish_autoware_status()
{
  AutowareStatus autoware_status;
  autoware_status.header.frame_id = "base_link";
  autoware_status.header.stamp = get_clock()->now();
  get_autoware_state_info(&autoware_status);
  get_control_mode_info(&autoware_status);
  get_gate_mode_info(&autoware_status);
  get_emergency_info(&autoware_status);
  RCLCPP_INFO(
    this->get_logger(), "[awapi_adapter]:AutowareStatus %i",
    autoware_status.header.stamp);
  pub_autoware_status_->publish(autoware_status);
}
void AutowareAutoStatusPublisher::get_autoware_state_info(AutowareStatus * status)
{
  std::string autoware_state = "test";
  status->autoware_state = "test";
}
void AutowareAutoStatusPublisher::get_control_mode_info(AutowareStatus * status)
{
  int32_t control_mode = 3;
  status->control_mode = control_mode;
}
void AutowareAutoStatusPublisher::get_gate_mode_info(AutowareStatus * status)
{
  int32_t gate_mode = 2;
  status->gate_mode = gate_mode;
}
void AutowareAutoStatusPublisher::get_emergency_info(AutowareStatus * status)
{
  std_msgs::msg::Bool is_emergency;
  is_emergency.data = true;
  status->emergency_stopped = is_emergency.data;
}
}  // namespace autoware_api
