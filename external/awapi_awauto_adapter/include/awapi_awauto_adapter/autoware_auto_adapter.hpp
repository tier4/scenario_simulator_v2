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

#ifndef AWAPI_AWAUTO_ADAPTER__AUTOWARE_AUTO_ADAPTER_HPP_
#define AWAPI_AWAUTO_ADAPTER__AUTOWARE_AUTO_ADAPTER_HPP_
#include <awapi_awauto_adapter/utility/visibility.h>
#include <autoware_api_msgs/msg/awapi_autoware_status.hpp>
#include <autoware_api_msgs/msg/awapi_vehicle_status.hpp>
#include <autoware_perception_msgs/msg/traffic_light_state_array.hpp>
#include <autoware_planning_msgs/msg/route.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

namespace autoware_api
{
class AutowareAutoAdapter : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr timer_;
  void global_timer();

  /** ---- AutowareEngage ------------------------------------------------------
   *  Topic: /awapi/autoware/put/engage
   * ------------------------------------------------------------------------ */
  using AutowareEngage = std_msgs::msg::Bool;
  rclcpp::Publisher<AutowareEngage>::SharedPtr pub_autoware_enage_;
  rclcpp::TimerBase::SharedPtr timer_engage_;
  void dummy_engage_autoware();

  /** ---- AutowareRoute -------------------------------------------------------
   *  Topic: /awapi/autoware/put/route
   * ------------------------------------------------------------------------ */
  using AutowareRoute = autoware_planning_msgs::msg::Route;
  rclcpp::Subscription<autoware_planning_msgs::msg::Route>::SharedPtr sub_route_;

  /** ---- AutowareStatus ------------------------------------------------------
   *  Topic: /awapi/autoware/get/status
   * ------------------------------------------------------------------------ */
  using AutowareStatus = autoware_api_msgs::msg::AwapiAutowareStatus;
  rclcpp::Publisher<AutowareStatus>::SharedPtr pub_autoware_status_;
  AutowareStatus autoware_status_;

public:
  AWAPI_AWAUTO_ADAPTER_PUBLIC
  explicit AutowareAutoAdapter(const rclcpp::NodeOptions &);
};
}  // namespace autoware_api
#endif  // AWAPI_AWAUTO_ADAPTER__AUTOWARE_AUTO_ADAPTER_HPP_
