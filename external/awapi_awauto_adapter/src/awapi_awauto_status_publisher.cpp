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

#include <awapi_awauto_adapter/awapi_awauto_status_publisher.hpp>

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
  autoware_status_ = AutowareStatus();
  autoware_status_.header.frame_id = "base_link";
  autoware_status_.header.stamp = get_clock()->now();
  autoware_status_.control_mode = 1;
  autoware_status_.gate_mode = 2;
  RCLCPP_INFO(
    this->get_logger(), "[awapi_adapter]:AutowareStatus %i",
    autoware_status_.header.stamp);
  pub_autoware_status_->publish(autoware_status_);
}
}  // namespace autoware_api
