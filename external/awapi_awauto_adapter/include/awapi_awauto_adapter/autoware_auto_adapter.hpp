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

#ifndef AWAPI_AWAUTO_ADAPTER__AUTOWARE_AUTO_ADAPTER_HPP_
#define AWAPI_AWAUTO_ADAPTER__AUTOWARE_AUTO_ADAPTER_HPP_
#include <awapi_awauto_adapter/utility/visibility.h>
#include <tf2_ros/transform_listener.h>
#include <autoware_api_msgs/msg/awapi_autoware_status.hpp>
#include <autoware_api_msgs/msg/awapi_vehicle_status.hpp>
#include <autoware_api_msgs/msg/lane_change_status.hpp>
#include <autoware_api_msgs/msg/obstacle_avoidance_status.hpp>
#include <autoware_perception_msgs/msg/dynamic_object_array.hpp>
#include <autoware_perception_msgs/msg/traffic_light_state_array.hpp>
#include <autoware_planning_msgs/msg/route.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <awapi_awauto_adapter/awapi_awauto_status_publisher.hpp>
#include <awapi_awauto_adapter/awapi_lane_change_status_publisher.hpp>
#include <awapi_awauto_adapter/awapi_obstacle_avoidance_status_publisher.hpp>
#include <awapi_awauto_adapter/awapi_vehicle_status_publisher.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <memory>

namespace autoware_api
{
class AutowareAutoAdapter : public rclcpp::Node
{
private:
  using Twist = geometry_msgs::msg::Twist;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Float32 = std_msgs::msg::Float32;
  using Bool = std_msgs::msg::Bool;
  using String = std_msgs::msg::Bool;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Pose = geometry_msgs::msg::Pose;
  using Route = autoware_planning_msgs::msg::Route;
  using Objects = autoware_perception_msgs::msg::DynamicObjectArray;
  using MapBin = std_msgs::msg::Empty;
  using PointXYZ = std_msgs::msg::Empty;
  /** @def
   *  AutowareStatus　Topic: /awapi/autoware/get/status
   */
  using AutowareStatus = autoware_api_msgs::msg::AwapiAutowareStatus;
  /** @def
   * VehicleStatus Topic: /awapi/vehicle/get/status
   */
  using VehicleStatus = autoware_api_msgs::msg::AwapiVehicleStatus;
  /** @def
   *  LaneChangeStatus Topic: /awapi/lane_change/get/status
   */
  using LaneChangeStatus = autoware_api_msgs::msg::LaneChangeStatus;
  /** @def
   * TrafficLightStatus Topic: /awapi/traffic_light/get/status
   */
  using TrafficLightStatus = autoware_perception_msgs::msg::TrafficLightStateArray;
  /**
   *  ObstacleAvoidanceStatus Topic: /awapi/traffic_light/get/status
   */
  using ObstacleAvoidanceStatus = autoware_api_msgs::msg::ObstacleAvoidanceStatus;

  // subscriber
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_initial_pose_;
  rclcpp::Subscription<Objects>::SharedPtr sub_detection_objects_;
  /// @todo make engage message
  rclcpp::Subscription<Bool>::SharedPtr sub_engage_;
  /// @todo make route message
  rclcpp::Subscription<Route>::SharedPtr sub_route_;
  /// @todo make twist message
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_twist_;
  /// @todo make steer message
  rclcpp::Subscription<Float32>::SharedPtr sub_steer_;
  /// @todo make steer message
  rclcpp::Subscription<Float32>::SharedPtr sub_steer_velocity_;
  /// @todo make limit velocity message
  rclcpp::Subscription<Float32>::SharedPtr sub_limit_velocity_;
  /// @todo make lane change approve message
  rclcpp::Subscription<Bool>::SharedPtr sub_lane_change_approve_;
  /// @todo make lane change force message message
  rclcpp::Subscription<Bool>::SharedPtr sub_lane_change_force_;
  /// @todo make lane change available message message
  rclcpp::Subscription<Bool>::SharedPtr sub_lane_change_available_;
  /// @todo make lane change ready message message
  rclcpp::Subscription<Bool>::SharedPtr sub_lane_change_ready_;
  /// @todo make initial twist message message
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_initial_twist_ptr_;
  /// @todo make checkpoint message
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_checkpoint_;
  /// @todo make goal message
  rclcpp::Subscription<Pose>::SharedPtr sub_goal_;
  /// @todo make traffic light message
  rclcpp::Subscription<TrafficLightStatus>::SharedPtr sub_traffic_light_;
  rclcpp::Subscription<Bool>::SharedPtr sub_obstacle_avoid_ready_;
  /// @todo make ros2 ll2 map type
  rclcpp::Subscription<MapBin>::SharedPtr sub_lanelet2_map_;
  /// @todo make ros2 pcl lib
  rclcpp::Subscription<PointXYZ>::SharedPtr sub_no_ground_pointcloud_;
  // publisher
  /// @todo make autoware status monitor
  rclcpp::Publisher<Bool>::SharedPtr pub_engage_;
  /// @todo make velocity controller
  rclcpp::Publisher<Float32>::SharedPtr pub_limit_velocity_;
  /// @todo make behavior planner
  rclcpp::Publisher<Route>::SharedPtr pub_route_;
  /// @todo make traffic light handler
  rclcpp::Publisher<TrafficLightStatus>::SharedPtr pub_traffic_light_status_;

  PoseStamped::SharedPtr pose_ptr_;
  TwistStamped::SharedPtr twist_ptr_;
  Float32::SharedPtr steer_ptr_;
  Float32::SharedPtr steer_velocity_ptr_;
  Float32::SharedPtr limit_velocity_ptr_;
  Bool::SharedPtr engage_ptr_;
  Route::SharedPtr route_ptr_;
  Bool::SharedPtr lane_change_approve_ptr_;
  Bool::SharedPtr lane_change_force_ptr_;
  Bool::SharedPtr lane_change_available_ptr_;
  Bool::SharedPtr lane_change_ready_ptr_;
  PoseStamped::SharedPtr initial_pose_ptr_;
  TwistStamped::SharedPtr initial_twist_ptr_;
  PoseStamped::SharedPtr checkpoint_ptr_;
  Pose::SharedPtr goal_ptr_;
  Objects::SharedPtr detection_object_ptr_;
  TrafficLightStatus::SharedPtr traffic_light_status_ptr_;
  Bool::SharedPtr obstacle_avoid_ready_ptr_;
  MapBin::SharedPtr lanelet2_map_ptr_;
  PointXYZ::SharedPtr point_cloud_ptr_;

  rclcpp::TimerBase::SharedPtr timer_callback_;
  /// @todo collect all necessary topic from AutowareAuto
  std::unique_ptr<AutowareAutoStatusPublisher> autoware_status_publisher_;
  /// @todo collect all necessary topic from AutowareAuto
  std::unique_ptr<AutowareVehicleStatusPublisher> vehicle_status_publisher_;
  /// @todo collect all necessary topic from AutowareAuto
  std::unique_ptr<AutowareLaneChangeStatusPublisher> lane_change_status_publisher_;
  /// @todo collect all necessary topic from AutowareAuto
  std::unique_ptr<AutowareObstacleAvoidanceStatusPublisher> obstacle_avoidance_status_publisher_;
  /**
   * @fn timer for each pulisher
   * @brief function to callback
   */
  void timer_callback();
  /**
   * @fn get current pose
   * @brief function to get current pose
   * @return geometry_msgs::msg::PoseStamped
   */
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
