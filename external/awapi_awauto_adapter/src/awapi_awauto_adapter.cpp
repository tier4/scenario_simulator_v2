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
#include <memory>

namespace autoware_api
{
AutowareAutoAdapter::AutowareAutoAdapter(const rclcpp::NodeOptions & options)
: rclcpp::Node("autoware_auto_adapter", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  sub_twist_ = create_subscription<TwistStamped>("/localization/twist",
      10, [&](const TwistStamped::SharedPtr msg_ptr) {twist_ptr_ = msg_ptr;});
  sub_steer_ = create_subscription<Float32>("input/steering",
      10, [&](const Float32::SharedPtr msg_ptr) {steer_ptr_ = msg_ptr;});

  pub_traffic_light_status_ = this->create_publisher<TrafficLightStatus>(
    "/awapi/traffic_light/get/status", 1);
  timer_callback_ =
    this->create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&AutowareAutoAdapter::timer_callback, this));
  autoware_status_publisher_ = std::make_unique<AutowareAutoStatusPublisher>(options);
  vehicle_status_publisher_ = std::make_unique<AutowareVehicleStatusPublisher>(options);
  lane_change_status_publisher_ = std::make_unique<AutowareLaneChangeStatusPublisher>(options);
  obstacle_avoidance_status_publisher_ = std::make_unique<AutowareObstacleAvoidanceStatusPublisher>(
    options);
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
    geometry_msgs::msg::PoseStamped ps;
    ps.header = transform.header;
    ps.pose.position.x = transform.transform.translation.x;
    ps.pose.position.y = transform.transform.translation.y;
    ps.pose.position.z = transform.transform.translation.z;
    ps.pose.orientation = transform.transform.rotation;
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(get_logger(), "cannot get self pose");
  }
}
}  // namespace autoware_api
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_api::AutowareAutoAdapter)
