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

#include <awapi_awauto_adapter/autoware_auto_adapter.hpp>
#include <rclcpp_components/register_node_macro.hpp>
using namespace std::chrono_literals;
namespace autoware_api
{
AutowareAutoAdapter::AutowareAutoAdapter(const rclcpp::NodeOptions & options)
: rclcpp::Node("autoware_auto_adapter", options)
{
  pub_autoware_status_ = this->create_publisher<AutowareStatus>("/awapi/autoware/get/status", 1);
  pub_vehicle_status_ = this->create_publisher<VehicleStatus>("/awapi/autoware/get/status", 1);
  pub_lane_change_status_ = this->create_publisher<LaneChangeStatus>(
    "/awapi/lane_change/get/status", 1);
  pub_traffic_light_status_ = this->create_publisher<TrafficLightStatus>(
    "/awapi/traffic_light/get/status", 1);
  timer_autoware_staus_ =
    this->create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&AutowareAutoAdapter::autoware_status_publisher, this));
  timer_vehicle_status_ =
    this->create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&AutowareAutoAdapter::vehicle_status_publisher, this));
  timer_lane_change_status_ =
    this->create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&AutowareAutoAdapter::lane_change_status_publisher, this));
  timer_traffic_light_status_ =
    this->create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&AutowareAutoAdapter::traffic_light_status_publisher, this));
}

void AutowareAutoAdapter::autoware_status_publisher()
{
  autoware_status_ = AutowareStatus();
  autoware_status_.autoware_state = "test";
  autoware_status_.control_mode = 1;
  autoware_status_.gate_mode = 2;
  RCLCPP_INFO(this->get_logger(), "[AutowareStatus]: %i", autoware_status_.control_mode);
  pub_autoware_status_->publish(autoware_status_);
}
void AutowareAutoAdapter::vehicle_status_publisher()
{
  VehicleStatus vehicle_status;
  vehicle_status.velocity = 0.1;
  pub_autoware_status_->publish(autoware_status_);
  RCLCPP_INFO(this->get_logger(), "[VehicleStatus]: %lf", vehicle_status.velocity);
  pub_vehicle_status_->publish(vehicle_status);
}

void AutowareAutoAdapter::lane_change_status_publisher()
{
  LaneChangeStatus lane_change_status;
  lane_change_status.force_lane_change_available = true;
  lane_change_status.lane_change_ready = true;
  RCLCPP_INFO(
    this->get_logger(), "[LaneChangeStatus]: %i",
    lane_change_status.force_lane_change_available);
  pub_lane_change_status_->publish(lane_change_status);
}

void AutowareAutoAdapter::traffic_light_status_publisher()
{
  TrafficLightStatus traffic_lights;
  traffic_lights.header.frame_id = "traffic_lights";
  RCLCPP_INFO(this->get_logger(), "[TrafficLight]: %s", traffic_lights.header.frame_id.c_str());
  pub_traffic_light_status_->publish(traffic_lights);
}
}  // namespace autoware_api
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_api::AutowareAutoAdapter)
