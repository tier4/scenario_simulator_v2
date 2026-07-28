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

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <type_traits>
#include <iomanip>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <traffic_simulator/replay/ego_bag_replayer.hpp>

namespace traffic_simulator
{
namespace
{
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

auto logger() -> rclcpp::Logger { return rclcpp::get_logger("traffic_simulator.ego_bag_replayer"); }

/// @note rosbag2_storage::SerializedBagMessage renamed `time_stamp` (Humble) to `recv_timestamp`
/// (Jazzy and later). Detect the available member at compile time so this file builds on both.
template <typename T, typename = void>
struct has_recv_timestamp : std::false_type
{
};
template <typename T>
struct has_recv_timestamp<T, std::void_t<decltype(std::declval<T>().recv_timestamp)>>
: std::true_type
{
};

template <typename T>
auto bagMessageTimestamp(const T & message)
{
  if constexpr (has_recv_timestamp<T>::value) {
    return message.recv_timestamp;
  } else {
    return message.time_stamp;
  }
}
}  // namespace

EgoBagReplayer::EgoBagReplayer(
  const std::string & bag_path, const double replay_start_time, const double replay_duration,
  const std::string & odometry_topic, const std::string & acceleration_topic)
: replay_duration_(replay_duration)
{
  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path;
  reader.open(storage_options);

  /// @note Same time base as PerceptionReproducerSensor::loadAllBagData(): the bag reception
  /// timestamp relative to the bag metadata starting_time, shifted by replay_start_time.
  const auto starting_time_ns =
    static_cast<int64_t>(reader.get_metadata().starting_time.time_since_epoch().count());
  const auto replay_start_time_ns = static_cast<int64_t>(replay_start_time * 1e9);

  rosbag2_storage::StorageFilter storage_filter;
  storage_filter.topics = {odometry_topic, acceleration_topic};
  reader.set_filter(storage_filter);

  rclcpp::Serialization<nav_msgs::msg::Odometry> odometry_serialization;
  rclcpp::Serialization<geometry_msgs::msg::AccelWithCovarianceStamped> accel_serialization;

  while (reader.has_next()) {
    const auto bag_message = reader.read_next();
    const auto time_ns = bagMessageTimestamp(*bag_message) - starting_time_ns - replay_start_time_ns;
    const rclcpp::SerializedMessage serialized_message(*bag_message->serialized_data);
    if (bag_message->topic_name == odometry_topic) {
      nav_msgs::msg::Odometry odometry;
      odometry_serialization.deserialize_message(&serialized_message, &odometry);
      odometries_.push_back({time_ns, odometry.pose.pose, odometry.twist.twist});
    } else if (bag_message->topic_name == acceleration_topic) {
      geometry_msgs::msg::AccelWithCovarianceStamped accel;
      accel_serialization.deserialize_message(&serialized_message, &accel);
      accelerations_.push_back({time_ns, accel.accel.accel});
    }
  }

  if (odometries_.empty()) {
    THROW_SIMULATION_ERROR(
      "No odometry message found on topic ", std::quoted(odometry_topic), " in rosbag ",
      std::quoted(bag_path), ".");
  }

  const auto by_time = [](const auto & a, const auto & b) { return a.time_ns < b.time_ns; };
  std::stable_sort(odometries_.begin(), odometries_.end(), by_time);
  std::stable_sort(accelerations_.begin(), accelerations_.end(), by_time);

  RCLCPP_INFO_STREAM(
    logger(), "Loaded " << odometries_.size() << " odometry samples covering scenario time "
                        << std::fixed << std::setprecision(3)
                        << odometries_.front().time_ns * 1e-9 << " - "
                        << odometries_.back().time_ns * 1e-9 << " [s] from "
                        << std::quoted(bag_path) << " (replay_start_time = " << replay_start_time
                        << " [s], replay_duration = " << replay_duration << " [s]).");
}

auto EgoBagReplayer::update(
  const double current_scenario_time, entity::EntityManager & entity_manager) -> void
{
  /// @note The scenario time is NaN until the interpreter confirms that every ego has been
  /// engaged, so the injection never interferes with Autoware initialization and route setting.
  if (switched_ or std::isnan(current_scenario_time) or current_scenario_time < 0.0) {
    return;
  }
  const auto ego_name = entity_manager.getFirstEgoName();
  if (not ego_name) {
    return;
  }
  auto & ego = entity_manager.getEgoEntity(*ego_name);
  if (current_scenario_time < replay_duration_) {
    const auto sample = interpolate(current_scenario_time);
    /// @note While this flag is set, API::updateEntitiesStatusInSim() requests the vehicle
    /// simulation to overwrite the vehicle model state instead of integrating control commands.
    ego.setControlledBySimulator(true);
    ego.setMapPose(sample.pose);
    ego.setTwist(sample.twist);
    ego.setAcceleration(sample.accel);
  } else {
    /// @note The vehicle model has been seeded with the replayed pose, velocity and acceleration
    /// on every frame, so just lowering the flag continues seamlessly.
    ego.setControlledBySimulator(false);
    switched_ = true;
    RCLCPP_INFO_STREAM(
      logger(), "Switched ego " << std::quoted(*ego_name) << " to closed-loop at scenario time "
                                << std::fixed << std::setprecision(3) << current_scenario_time
                                << " [s].");
  }
}

auto EgoBagReplayer::interpolate(const double scenario_time) const -> Sample
{
  const auto time_ns = static_cast<int64_t>(scenario_time * 1e9);

  Sample sample;
  {
    const auto upper = std::upper_bound(
      odometries_.begin(), odometries_.end(), time_ns,
      [](int64_t value, const auto & element) { return value < element.time_ns; });
    if (upper == odometries_.begin()) {
      sample.pose = odometries_.front().pose;
      sample.twist = odometries_.front().twist;
    } else if (upper == odometries_.end()) {
      sample.pose = odometries_.back().pose;
      sample.twist = odometries_.back().twist;
    } else {
      const auto & next = *upper;
      const auto & previous = *std::prev(upper);
      const auto t = static_cast<double>(time_ns - previous.time_ns) /
                     static_cast<double>(next.time_ns - previous.time_ns);
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
      accelerations_.begin(), accelerations_.end(), time_ns,
      [](int64_t value, const auto & element) { return value < element.time_ns; });
    if (upper == accelerations_.begin()) {
      sample.accel = accelerations_.front().accel;
    } else if (upper == accelerations_.end()) {
      sample.accel = accelerations_.back().accel;
    } else {
      const auto & next = *upper;
      const auto & previous = *std::prev(upper);
      const auto t = static_cast<double>(time_ns - previous.time_ns) /
                     static_cast<double>(next.time_ns - previous.time_ns);
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
}  // namespace traffic_simulator
