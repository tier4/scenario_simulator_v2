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

#ifndef AWAPI_AWAUTO_ADAPTER__AWAPI_LANE_CHANGE_STATUS_PUBLISHER_HPP_
#define AWAPI_AWAUTO_ADAPTER__AWAPI_LANE_CHANGE_STATUS_PUBLISHER_HPP_
#include <awapi_awauto_adapter/utility/visibility.h>
#include <autoware_api_msgs/msg/lane_change_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <memory>

namespace autoware_api
{
class AutowareLaneChangeStatusPublisher : public rclcpp::Node
{
private:
  /** ---- LaneChangeStatus ------------------------------------------------------
   *  Topic: /awapi/lane_change/get/status
   * ------------------------------------------------------------------------ */
  using LaneChangeStatus = autoware_api_msgs::msg::LaneChangeStatus;
  rclcpp::Publisher<LaneChangeStatus>::SharedPtr pub_lane_change_status_;
  rclcpp::TimerBase::SharedPtr timer_lane_change_status_;

public:
  AWAPI_AWAUTO_ADAPTER_PUBLIC
  void publish_lane_change_status();
  explicit AutowareLaneChangeStatusPublisher(const rclcpp::NodeOptions &);
};
}  // namespace autoware_api
#endif  // AWAPI_AWAUTO_ADAPTER__AWAPI_LANE_CHANGE_STATUS_PUBLISHER_HPP_
