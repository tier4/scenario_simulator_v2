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

#include <awapi_awauto_adapter/awapi_lane_change_status_publisher.hpp>
#include <string>
using LaneChangeStatus = autoware_api_msgs::msg::LaneChangeStatus;

namespace autoware_api
{
AutowareLaneChangeStatusPublisher::AutowareLaneChangeStatusPublisher(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("awapi_lane_change_status_publisher", options)
{
  // publisher
  pub_lane_change_status_ =
    this->create_publisher<LaneChangeStatus>("/awapi/lane_change/get/status", 1);
}
void AutowareLaneChangeStatusPublisher::publish_lane_change_status()
{
  LaneChangeStatus lane_change_status;
  lane_change_status.header.frame_id = "map";
  lane_change_status.header.stamp = get_clock()->now();
  lane_change_status.force_lane_change_available = true;
  lane_change_status.lane_change_ready = true;
  RCLCPP_INFO(this->get_logger(), "LaneChangeStatus %i", lane_change_status.header.stamp);
  pub_lane_change_status_->publish(lane_change_status);
}

}  // namespace autoware_api
