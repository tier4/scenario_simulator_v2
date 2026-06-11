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

#ifndef ROLLOUT_TEST_RUNNER__EGO_REPLAY_SOURCE_HPP_
#define ROLLOUT_TEST_RUNNER__EGO_REPLAY_SOURCE_HPP_

#include <cstdint>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/time.hpp>
#include <string>
#include <utility>
#include <vector>

namespace rollout_test_runner
{
/// Loads ego kinematic states from a rosbag and provides time-interpolated samples.
class EgoReplaySource
{
public:
  struct Sample
  {
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Twist twist;
    geometry_msgs::msg::Accel accel;
  };

  explicit EgoReplaySource(
    const std::string & bag_path, const std::string & odometry_topic,
    const std::string & acceleration_topic);

  auto startTime() const -> rclcpp::Time;

  auto endTime() const -> rclcpp::Time;

  /// Returns the sample interpolated at the given bag time, clamped to [startTime, endTime].
  auto interpolate(const rclcpp::Time & bag_time) const -> Sample;

private:
  struct OdometrySample
  {
    int64_t stamp_ns;
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Twist twist;
  };

  struct AccelerationSample
  {
    int64_t stamp_ns;
    geometry_msgs::msg::Accel accel;
  };

  std::vector<OdometrySample> odometries_;
  std::vector<AccelerationSample> accelerations_;
};
}  // namespace rollout_test_runner

#endif  // ROLLOUT_TEST_RUNNER__EGO_REPLAY_SOURCE_HPP_
