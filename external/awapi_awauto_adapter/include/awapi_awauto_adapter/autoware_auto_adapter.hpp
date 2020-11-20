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
#include <autoware_api_msgs/msg/lane_change_status.hpp>
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
  /** ---- AutowareStatus ------------------------------------------------------
   *  Topic: /awapi/autoware/get/status
   * ------------------------------------------------------------------------ */
  using AutowareStatus = autoware_api_msgs::msg::AwapiAutowareStatus;
  rclcpp::Publisher<AutowareStatus>::SharedPtr pub_autoware_status_;
  AutowareStatus autoware_status_;
  rclcpp::TimerBase::SharedPtr timer_autoware_staus_;
  void publish_autoware_status();

  /** ---- VehicleStatus -------------------------------------------------------
   *  Topic: /awapi/vehicle/get/status
   * ------------------------------------------------------------------------ */
  using VehicleStatus = autoware_api_msgs::msg::AwapiVehicleStatus;
  rclcpp::Publisher<VehicleStatus>::SharedPtr pub_vehicle_status_;
  VehicleStatus vehicle_status_;
  rclcpp::TimerBase::SharedPtr timer_vehicle_status_;
  void publish_vehicle_status();

  /** ---- LaneChangeStatus ------------------------------------------------------
   *  Topic: /awapi/lane_change/get/status
   * ------------------------------------------------------------------------ */
  using LaneChangeStatus = autoware_api_msgs::msg::LaneChangeStatus;
  rclcpp::Publisher<LaneChangeStatus>::SharedPtr pub_lane_change_status_;
  LaneChangeStatus lane_change_status_;
  rclcpp::TimerBase::SharedPtr timer_lane_change_status_;
  void publish_lane_change_status();

  /** ---- TrafficLightStatus --------------------------------------------------
   *  Topic: /awapi/traffic_light/get/status
   * ------------------------------------------------------------------------ */
  using TrafficLightStatus = autoware_perception_msgs::msg::TrafficLightStateArray;
  rclcpp::Publisher<TrafficLightStatus>::SharedPtr pub_traffic_light_status_;
  TrafficLightStatus traffic_lights_;
  rclcpp::TimerBase::SharedPtr timer_traffic_light_status_;
  void publish_traffic_light_status();

public:
  AWAPI_AWAUTO_ADAPTER_PUBLIC
  explicit AutowareAutoAdapter(const rclcpp::NodeOptions &);
};
}  // namespace autoware_api
#endif  // AWAPI_AWAUTO_ADAPTER__AUTOWARE_AUTO_ADAPTER_HPP_
