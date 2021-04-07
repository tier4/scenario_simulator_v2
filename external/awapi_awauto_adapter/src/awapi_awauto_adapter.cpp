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

#include <awapi_awauto_adapter/autoware_auto_adapter.hpp>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>
using std::placeholders::_1;

namespace autoware_api
{
AutowareAutoAdapter::AutowareAutoAdapter(const rclcpp::NodeOptions & options)
: rclcpp::Node("autoware_auto_adapter", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  // publisher
  pub_engage_ = create_publisher<Bool>("/awapi/vehicle/put/engage", 1);
  pub_limit_velocity_ = create_publisher<Float32>("/awapi/vehicle/put/limit_velocity", 1);
  pub_route_ = create_publisher<Route>("/awapi/autoware/put/route", 1);
  pub_traffic_light_status_ = create_publisher<TrafficLightStatus>("output/traffic_light", 1);

  // subscriber
  sub_twist_ = create_subscription<TwistStamped>(
    "/localization/twist", 1, [&](const TwistStamped::SharedPtr msg_ptr) { twist_ptr_ = msg_ptr; });

  sub_steer_ = create_subscription<Float32>(
    "input/steering", 1, [&](const Float32::SharedPtr msg_ptr) { steer_ptr_ = msg_ptr; });

  sub_steer_velocity_ = create_subscription<Float32>(
    "input/steering_velocity", 1,
    [&](const Float32::SharedPtr msg_ptr) { steer_velocity_ptr_ = msg_ptr; });

  sub_limit_velocity_ =
    create_subscription<Float32>("input/limit_velocity", 1, [&](const Float32::SharedPtr msg_ptr) {
      pub_limit_velocity_->publish(*(limit_velocity_ptr_ = msg_ptr));
    });

  sub_engage_ = create_subscription<Bool>(
    "/vehicle/get/engage", 1,
    [&](const Bool::SharedPtr msg_ptr) { pub_engage_->publish(*(engage_ptr_ = msg_ptr)); });

  sub_route_ = create_subscription<Route>("input/route", 1, [&](const Route::SharedPtr msg_ptr) {
    pub_route_->publish(*(route_ptr_ = msg_ptr));
  });

  sub_lane_change_approve_ = create_subscription<Bool>(
    "input/lane_change_approve", 1,
    [&](const Bool::SharedPtr msg_ptr) { lane_change_approve_ptr_ = msg_ptr; });

  sub_lane_change_force_ = create_subscription<Bool>(
    "input/lane_change_force", 1,
    [&](const Bool::SharedPtr msg_ptr) { lane_change_force_ptr_ = msg_ptr; });

  sub_lane_change_available_ = create_subscription<Bool>(
    "input/lane_change_avaiable", 1,
    [&](const Bool::SharedPtr msg_ptr) { lane_change_available_ptr_ = msg_ptr; });

  sub_lane_change_ready_ = create_subscription<Bool>(
    "input/lane_change_ready", 1,
    [&](const Bool::SharedPtr msg_ptr) { lane_change_ready_ptr_ = msg_ptr; });

  sub_initial_pose_ = create_subscription<PoseStamped>(
    "/initialpose", 1, [&](const PoseStamped::SharedPtr msg_ptr) { initial_pose_ptr_ = msg_ptr; });

  sub_initial_twist_ptr_ = create_subscription<TwistStamped>(
    "/initialtwist", 1,
    [&](const TwistStamped::SharedPtr msg_ptr) { initial_twist_ptr_ = msg_ptr; });

  sub_checkpoint_ = create_subscription<PoseStamped>(
    "/planning/mission_planning/checkpoint", 1,
    [&](const PoseStamped::SharedPtr msg_ptr) { checkpoint_ptr_ = msg_ptr; });

  sub_goal_ = create_subscription<Pose>(
    "/planning/mission_planning/goal", 1,
    [&](const Pose::SharedPtr msg_ptr) { goal_ptr_ = msg_ptr; });

  sub_detection_objects_ = create_subscription<Objects>(
    "/perception/object_recognition/detection/objects", 1,
    [&](const Objects::SharedPtr msg_ptr) { detection_object_ptr_ = msg_ptr; });

  sub_obstacle_avoid_ready_ = create_subscription<Bool>(
    "input/sub_obstacle_avoid_ready", 1,
    [&](const Bool::SharedPtr msg_ptr) { obstacle_avoid_ready_ptr_ = msg_ptr; });

  sub_lanelet2_map_ = create_subscription<MapBin>(
    "/map/vector_map", 1, [&](const MapBin::SharedPtr msg_ptr) { lanelet2_map_ptr_ = msg_ptr; });

  sub_no_ground_pointcloud_ = create_subscription<PointXYZ>(
    "/sensing/lidar/no_ground_pointcloud", 1,
    [&](const PointXYZ::SharedPtr msg_ptr) { point_cloud_ptr_ = msg_ptr; });

  sub_traffic_light_ = create_subscription<TrafficLightStatus>(
    "/perception/traffic_light_recognition/traffic_light_states", 1,
    [&](const TrafficLightStatus::SharedPtr msg_ptr) { traffic_light_status_ptr_ = msg_ptr; });

  timer_callback_ = create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&AutowareAutoAdapter::timer_callback, this));

  autoware_status_publisher_ = std::make_unique<AutowareAutoStatusPublisher>(options);
  vehicle_status_publisher_ = std::make_unique<AutowareVehicleStatusPublisher>(options);
  lane_change_status_publisher_ = std::make_unique<AutowareLaneChangeStatusPublisher>(options);
  obstacle_avoidance_status_publisher_ =
    std::make_unique<AutowareObstacleAvoidanceStatusPublisher>(options);
}

void AutowareAutoAdapter::timer_callback()
{
  get_current_pose();
  autoware_status_publisher_->publish_autoware_status();
  vehicle_status_publisher_->publish_vehicle_status();
  lane_change_status_publisher_->publish_lane_change_status();
  obstacle_avoidance_status_publisher_->publish_obstacle_avoidance_status();
}

void AutowareAutoAdapter::get_current_pose()
{
  try {
    tf2::TimePoint time_point = tf2::TimePoint(std::chrono::seconds(0));
    geometry_msgs::msg::TransformStamped transform =
      tf_buffer_.lookupTransform("map", "base_link", time_point);
    PoseStamped ps;
    ps.header = transform.header;
    ps.pose.position.x = transform.transform.translation.x;
    ps.pose.position.y = transform.transform.translation.y;
    ps.pose.position.z = transform.transform.translation.z;
    ps.pose.orientation = transform.transform.rotation;
    pose_ptr_ = std::make_shared<PoseStamped>(ps);
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(get_logger(), "cannot get self pose");
  }
}
}  // namespace autoware_api
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_api::AutowareAutoAdapter)
