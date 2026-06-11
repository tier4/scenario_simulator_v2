// Copyright 2026 TIER IV, Inc. All rights reserved.
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

#include <algorithm>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <iomanip>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/serialization.hpp>
#include <rollout_test_runner/ego_replay_source.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rollout_test_runner
{
namespace
{
auto toNanoseconds(const builtin_interfaces::msg::Time & stamp) -> int64_t
{
  return static_cast<int64_t>(stamp.sec) * 1000000000 + stamp.nanosec;
}

auto lerp(double a, double b, double t) -> double { return a + (b - a) * t; }

auto slerp(
  const geometry_msgs::msg::Quaternion & a, const geometry_msgs::msg::Quaternion & b, double t)
  -> geometry_msgs::msg::Quaternion
{
  tf2::Quaternion qa, qb;
  tf2::fromMsg(a, qa);
  tf2::fromMsg(b, qb);
  return tf2::toMsg(qa.slerp(qb, t));
}
}  // namespace

EgoReplaySource::EgoReplaySource(
  const std::string & bag_path, const std::string & odometry_topic,
  const std::string & acceleration_topic)
{
  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path;
  reader.open(storage_options, rosbag2_cpp::ConverterOptions());
  reader.set_filter(rosbag2_storage::StorageFilter{{odometry_topic, acceleration_topic}});

  rclcpp::Serialization<nav_msgs::msg::Odometry> odometry_serialization;
  rclcpp::Serialization<geometry_msgs::msg::AccelWithCovarianceStamped> accel_serialization;

  while (reader.has_next()) {
    const auto bag_message = reader.read_next();
    const rclcpp::SerializedMessage serialized_message(*bag_message->serialized_data);
    if (bag_message->topic_name == odometry_topic) {
      nav_msgs::msg::Odometry odometry;
      odometry_serialization.deserialize_message(&serialized_message, &odometry);
      odometries_.push_back(
        {toNanoseconds(odometry.header.stamp), odometry.pose.pose, odometry.twist.twist});
    } else if (bag_message->topic_name == acceleration_topic) {
      geometry_msgs::msg::AccelWithCovarianceStamped accel;
      accel_serialization.deserialize_message(&serialized_message, &accel);
      accelerations_.push_back({toNanoseconds(accel.header.stamp), accel.accel.accel});
    }
  }

  if (odometries_.empty()) {
    THROW_SIMULATION_ERROR(
      "No odometry message found on topic ", std::quoted(odometry_topic), " in rosbag ",
      std::quoted(bag_path), ".");
  }

  const auto by_stamp = [](const auto & a, const auto & b) { return a.stamp_ns < b.stamp_ns; };
  std::stable_sort(odometries_.begin(), odometries_.end(), by_stamp);
  std::stable_sort(accelerations_.begin(), accelerations_.end(), by_stamp);
}

auto EgoReplaySource::startTime() const -> rclcpp::Time
{
  return rclcpp::Time(odometries_.front().stamp_ns, RCL_ROS_TIME);
}

auto EgoReplaySource::endTime() const -> rclcpp::Time
{
  return rclcpp::Time(odometries_.back().stamp_ns, RCL_ROS_TIME);
}

auto EgoReplaySource::interpolate(const rclcpp::Time & bag_time) const -> Sample
{
  const auto stamp_ns = bag_time.nanoseconds();

  Sample sample;
  {
    const auto upper = std::upper_bound(
      odometries_.begin(), odometries_.end(), stamp_ns,
      [](int64_t value, const auto & element) { return value < element.stamp_ns; });
    if (upper == odometries_.begin()) {
      sample.pose = odometries_.front().pose;
      sample.twist = odometries_.front().twist;
    } else if (upper == odometries_.end()) {
      sample.pose = odometries_.back().pose;
      sample.twist = odometries_.back().twist;
    } else {
      const auto & next = *upper;
      const auto & previous = *std::prev(upper);
      const auto t = static_cast<double>(stamp_ns - previous.stamp_ns) /
                     static_cast<double>(next.stamp_ns - previous.stamp_ns);
      sample.pose.position.x = lerp(previous.pose.position.x, next.pose.position.x, t);
      sample.pose.position.y = lerp(previous.pose.position.y, next.pose.position.y, t);
      sample.pose.position.z = lerp(previous.pose.position.z, next.pose.position.z, t);
      sample.pose.orientation = slerp(previous.pose.orientation, next.pose.orientation, t);
      sample.twist.linear.x = lerp(previous.twist.linear.x, next.twist.linear.x, t);
      sample.twist.linear.y = lerp(previous.twist.linear.y, next.twist.linear.y, t);
      sample.twist.linear.z = lerp(previous.twist.linear.z, next.twist.linear.z, t);
      sample.twist.angular.x = lerp(previous.twist.angular.x, next.twist.angular.x, t);
      sample.twist.angular.y = lerp(previous.twist.angular.y, next.twist.angular.y, t);
      sample.twist.angular.z = lerp(previous.twist.angular.z, next.twist.angular.z, t);
    }
  }

  if (not accelerations_.empty()) {
    const auto upper = std::upper_bound(
      accelerations_.begin(), accelerations_.end(), stamp_ns,
      [](int64_t value, const auto & element) { return value < element.stamp_ns; });
    if (upper == accelerations_.begin()) {
      sample.accel = accelerations_.front().accel;
    } else if (upper == accelerations_.end()) {
      sample.accel = accelerations_.back().accel;
    } else {
      const auto & next = *upper;
      const auto & previous = *std::prev(upper);
      const auto t = static_cast<double>(stamp_ns - previous.stamp_ns) /
                     static_cast<double>(next.stamp_ns - previous.stamp_ns);
      sample.accel.linear.x = lerp(previous.accel.linear.x, next.accel.linear.x, t);
      sample.accel.linear.y = lerp(previous.accel.linear.y, next.accel.linear.y, t);
      sample.accel.linear.z = lerp(previous.accel.linear.z, next.accel.linear.z, t);
      sample.accel.angular.x = lerp(previous.accel.angular.x, next.accel.angular.x, t);
      sample.accel.angular.y = lerp(previous.accel.angular.y, next.accel.angular.y, t);
      sample.accel.angular.z = lerp(previous.accel.angular.z, next.accel.angular.z, t);
    }
  }

  return sample;
}
}  // namespace rollout_test_runner
