// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include <cmath>
#include <geometry/quaternion/slerp.hpp>
#include <geometry/vector3/operator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <simple_sensor_simulator/sensor_simulation/perception_reproducer_sensor/perception_reproducer_sensor.hpp>
#include <stdexcept>

namespace simple_sensor_simulator
{

auto TFStreamFromOdometry::pushMessage(
  double time_s, const std::shared_ptr<rcutils_uint8_array_t> & data) -> void
{
  data_.emplace_back(time_s, deserialize(data));
}

auto TFStreamFromOdometry::broadcastTf(double time_s, const rclcpp::Time & ros_time)
  -> geometry_msgs::msg::Pose
{
  geometry_msgs::msg::Pose pose;

  if (time_s <= data_.front().first) {
    pose = data_.front().second.pose.pose;
  } else if (time_s >= data_.back().first) {
    index_ = data_.size() - 1;
    pose = data_.back().second.pose.pose;
  } else {
    while (index_ + 1 < data_.size() && data_[index_ + 1].first <= time_s) {
      ++index_;
    }

    const auto & prev = data_[index_].second.pose.pose;
    const auto & next = data_[index_ + 1].second.pose.pose;

    const double span = data_[index_ + 1].first - data_[index_].first;
    const double ratio = (span > 0.0) ? (time_s - data_[index_].first) / span : 0.0;

    using math::geometry::operator+;
    using math::geometry::operator-;
    using math::geometry::operator*;

    pose.position = prev.position * (1.0 - ratio) + next.position * ratio;
    pose.orientation = math::geometry::slerp(prev.orientation, next.orientation, ratio);
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = ros_time;
  transform.header.frame_id = "map";
  transform.child_frame_id = frame_id_;
  transform.transform.translation.x = pose.position.x;
  transform.transform.translation.y = pose.position.y;
  transform.transform.translation.z = pose.position.z;
  transform.transform.rotation = pose.orientation;
  tf_broadcaster_->sendTransform(transform);

  return pose;
}

PerceptionReproducerSensor::PerceptionReproducerSensor(
  const std::string & bag_path, double start_time_s,
  rclcpp::Publisher<DetectedObjects>::SharedPtr publisher, rclcpp::Node & node)
: logger_(node.get_logger()),
  detected_objects_stream_(detected_objects_topic_, publisher),
  trajectory_stream_(
    trajectory_topic_,
    node.create_publisher<Trajectory>("/simulation/replay/trajectory", 1)),
  odometry_stream_(odometry_topic_, "replay_base_link", node),
  vehicle_marker_pub_(node.create_publisher<visualization_msgs::msg::MarkerArray>(
    "/simulation/replay/vehicle_marker", 1))
{
  loadAllBagData(bag_path, start_time_s);
}

auto PerceptionReproducerSensor::loadAllBagData(const std::string & bag_path, double start_time_s)
  -> void
{
  RCLCPP_INFO(logger_, "Loading bag: %s (start_time: %.3f s)", bag_path.c_str(), start_time_s);

  auto reader = std::make_unique<rosbag2_cpp::Reader>();
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path;
  storage_options.storage_id = "mcap";
  reader->open(storage_options);

  const rclcpp::Time first_time(
    reader->get_metadata().starting_time.time_since_epoch().count(), RCL_ROS_TIME);

  rosbag2_storage::StorageFilter filter;
  filter.topics = {detected_objects_topic_, trajectory_topic_, odometry_topic_};
  reader->set_filter(filter);

  while (reader->has_next()) {
    try {
      auto bag_message = reader->read_next();
      const rclcpp::Time msg_time(bag_message->time_stamp, RCL_ROS_TIME);
      const double shifted_time_s = (msg_time - first_time).seconds() - start_time_s;
      if (shifted_time_s < 0.0) continue;

      detected_objects_stream_.tryPushMessage(bag_message, shifted_time_s);
      trajectory_stream_.tryPushMessage(bag_message, shifted_time_s);
      odometry_stream_.tryPushMessage(bag_message, shifted_time_s);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Error reading message: %s", e.what());
    }
  }
}

auto PerceptionReproducerSensor::publishVehicleMarker(
  const geometry_msgs::msg::Pose & pose, const rclcpp::Time & ros_time) -> void
{
  const auto make_marker = [&](int id, int type) {
    visualization_msgs::msg::Marker m;
    m.header.stamp = ros_time;
    m.header.frame_id = "map";
    m.ns = "replay_vehicle";
    m.id = id;
    m.type = type;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = pose;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);
    return m;
  };

  auto cube = make_marker(0, visualization_msgs::msg::Marker::CUBE);
  cube.scale.x = 4.5;
  cube.scale.y = 1.8;
  cube.scale.z = 1.5;
  cube.color.r = 1.0f;
  cube.color.g = 0.5f;
  cube.color.b = 0.0f;
  cube.color.a = 0.5f;

  auto arrow = make_marker(1, visualization_msgs::msg::Marker::ARROW);
  arrow.scale.x = 3.0;
  arrow.scale.y = 0.3;
  arrow.scale.z = 0.3;
  arrow.color.r = 1.0f;
  arrow.color.g = 0.5f;
  arrow.color.b = 0.0f;
  arrow.color.a = 0.8f;

  auto text = make_marker(2, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  text.pose.position.z += 2.0;
  text.scale.z = 0.8;
  text.color.r = 1.0f;
  text.color.g = 1.0f;
  text.color.b = 1.0f;
  text.color.a = 1.0f;
  text.text = "Replay Vehicle";

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers = {cube, arrow, text};
  vehicle_marker_pub_->publish(marker_array);
}

auto PerceptionReproducerSensor::update(
  double current_scenario_time, const rclcpp::Time & current_ros_time) -> void
{
  if (std::isnan(current_scenario_time) || current_scenario_time < 0.0) {
    return;
  }

  if (detected_objects_stream_.done() && trajectory_stream_.done()) {
    return;
  }

  if (not odometry_stream_.empty()) {
    const auto pose = odometry_stream_.broadcastTf(current_scenario_time, current_ros_time);
    publishVehicleMarker(pose, current_ros_time);
  }

  detected_objects_stream_.publishUpTo(current_scenario_time, current_ros_time);
  trajectory_stream_.publishUpTo(current_scenario_time, current_ros_time);
}

auto PerceptionReproducerSensor::reset() -> void
{
  detected_objects_stream_.reset();
  trajectory_stream_.reset();
  odometry_stream_.reset();
}

}  // namespace simple_sensor_simulator
