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
#include <autoware_api_msgs/msg/obstacle_avoidance_status.hpp>
#include <autoware_perception_msgs/msg/traffic_light_state_array.hpp>
#include <autoware_planning_msgs/msg/route.hpp>
#include <awapi_awauto_adapter/awapi_awauto_status_publisher.hpp>
#include <awapi_awauto_adapter/awapi_lane_change_status_publisher.hpp>
#include <awapi_awauto_adapter/awapi_obstacle_avoidance_status_publisher.hpp>
#include <awapi_awauto_adapter/awapi_vehicle_status_publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>

namespace autoware_api
{
class AutowareAutoAdapter : public rclcpp::Node
{
private:
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Float32 = std_msgs::msg::Float32;
  using Bool = std_msgs::msg::Bool;
  TwistStamped::SharedPtr twist_ptr_;
  Float32::SharedPtr steer_ptr_;
  Bool::ConstSharedPtr lane_change_available_ptr;
  Bool::ConstSharedPtr lane_change_ready_ptr;
  Bool::ConstSharedPtr obstacle_avoid_ready_ptr;
  // subscriber
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_twist_;
  rclcpp::Subscription<Float32>::SharedPtr sub_steer_;
  rclcpp::Subscription<Bool>::SharedPtr sub_lane_change_available_;
  rclcpp::Subscription<Bool>::SharedPtr sub_lane_change_ready_;
  rclcpp::Subscription<Bool>::SharedPtr sub_obstacle_avoid_ready_;

  // publish
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_callback_;
  /** ---- AutowareStatus ------------------------------------------------------
   *  Topic: /awapi/autoware/get/status
   * ------------------------------------------------------------------------ */
  using AutowareStatus = autoware_api_msgs::msg::AwapiAutowareStatus;
  rclcpp::Publisher<AutowareStatus>::SharedPtr pub_autoware_status_;
  std::unique_ptr<AutowareAutoStatusPublisher> autoware_status_publisher_;

  /** ---- VehicleStatus -------------------------------------------------------
   *  Topic: /awapi/vehicle/get/status
   * ------------------------------------------------------------------------ */
  using VehicleStatus = autoware_api_msgs::msg::AwapiVehicleStatus;
  rclcpp::Publisher<VehicleStatus>::SharedPtr pub_vehicle_status_;
  std::unique_ptr<AutowareVehicleStatusPublisher> vehicle_status_publisher_;

  /** ---- LaneChangeStatus ------------------------------------------------------
   *  Topic: /awapi/lane_change/get/status
   * ------------------------------------------------------------------------ */
  using LaneChangeStatus = autoware_api_msgs::msg::LaneChangeStatus;
  rclcpp::Publisher<LaneChangeStatus>::SharedPtr pub_lane_change_status_;
  std::unique_ptr<AutowareLaneChangeStatusPublisher> lane_change_status_publisher_;

  /** ---- TrafficLightStatus --------------------------------------------------
   *  Topic: /awapi/traffic_light/get/status
   * ------------------------------------------------------------------------ */
  using TrafficLightStatus = autoware_perception_msgs::msg::TrafficLightStateArray;
  rclcpp::Publisher<TrafficLightStatus>::SharedPtr pub_traffic_light_status_;
  TrafficLightStatus traffic_lights_;
  rclcpp::TimerBase::SharedPtr timer_traffic_light_status_;
  void publish_traffic_light_status();

  /** ---- ObstacleAvoidanceStatus --------------------------------------------------
   *  Topic: /awapi/traffic_light/get/status
   * ------------------------------------------------------------------------ */
  using ObstacleAvoidanceStatus = autoware_api_msgs::msg::ObstacleAvoidanceStatus;
  rclcpp::Publisher<TrafficLightStatus>::SharedPtr pub_obstacle_avoidance_status_;
  std::unique_ptr<AutowareObstacleAvoidanceStatusPublisher> obstacle_avoidance_status_publisher_;
  void get_current_pose();

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

public:
  AWAPI_AWAUTO_ADAPTER_PUBLIC
  explicit AutowareAutoAdapter(const rclcpp::NodeOptions &);
};
}  // namespace autoware_api
#endif  // AWAPI_AWAUTO_ADAPTER__AUTOWARE_AUTO_ADAPTER_HPP_
