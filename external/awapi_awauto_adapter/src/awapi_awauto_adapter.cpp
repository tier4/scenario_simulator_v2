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

#include <awapi_awauto_adapter/autoware_auto_adapter.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <memory>

namespace autoware_api
{
AutowareAutoAdapter::AutowareAutoAdapter(const rclcpp::NodeOptions & options)
: rclcpp::Node("autoware_auto_adapter", options)
{
  pub_autoware_status_ = this->create_publisher<AutowareStatus>("/awapi/autoware/get/status", 1);
  pub_vehicle_status_ = this->create_publisher<VehicleStatus>("/awapi/vehicle/get/status", 1);
  pub_lane_change_status_ = this->create_publisher<LaneChangeStatus>(
    "/awapi/lane_change/get/status", 1);
  pub_traffic_light_status_ = this->create_publisher<TrafficLightStatus>(
    "/awapi/traffic_light/get/status", 1);
  timer_callback_ =
    this->create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&AutowareAutoAdapter::timer_callback, this));
  timer_vehicle_status_ =
    this->create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&AutowareAutoAdapter::publish_vehicle_status, this));
  timer_lane_change_status_ =
    this->create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&AutowareAutoAdapter::publish_lane_change_status, this));
  timer_traffic_light_status_ =
    this->create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&AutowareAutoAdapter::publish_traffic_light_status, this));
  autoware_state_publisher_ = std::make_unique<AutowareAutoStatusPublisher>(options);
}

void AutowareAutoAdapter::timer_callback()
{
  autoware_state_publisher_->publish_autoware_status();
}

void AutowareAutoAdapter::publish_vehicle_status()
{
  vehicle_status_ = VehicleStatus();
  vehicle_status_.header.frame_id = "base_link";
  vehicle_status_.header.stamp = get_clock()->now();
  vehicle_status_.velocity = 0.1;
  pub_vehicle_status_->publish(vehicle_status_);
  RCLCPP_INFO(this->get_logger(), " VehicleStatus %i",
    vehicle_status_.header.stamp);
}

void AutowareAutoAdapter::publish_lane_change_status()
{
  lane_change_status_ = LaneChangeStatus();
  lane_change_status_.header.frame_id = "map";
  lane_change_status_.header.stamp = get_clock()->now();
  lane_change_status_.force_lane_change_available = true;
  lane_change_status_.lane_change_ready = true;
  RCLCPP_INFO(
    this->get_logger(), "LaneChangeStatus %i", lane_change_status_.header.stamp);
  pub_lane_change_status_->publish(lane_change_status_);
}

void AutowareAutoAdapter::publish_traffic_light_status()
{
  traffic_lights_ = TrafficLightStatus();
  traffic_lights_.header.frame_id = "map";
  traffic_lights_.header.stamp = get_clock()->now();
  RCLCPP_INFO(
    this->get_logger(), "TrafficLightStatus %i", traffic_lights_.header.stamp);
  pub_traffic_light_status_->publish(traffic_lights_);
}
}  // namespace autoware_api
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_api::AutowareAutoAdapter)
