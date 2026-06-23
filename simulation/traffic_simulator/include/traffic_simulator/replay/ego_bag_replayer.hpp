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

#ifndef TRAFFIC_SIMULATOR__REPLAY__EGO_BAG_REPLAYER_HPP_
#define TRAFFIC_SIMULATOR__REPLAY__EGO_BAG_REPLAYER_HPP_

#include <cstdint>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <traffic_simulator/entity/entity_manager.hpp>
#include <vector>

namespace traffic_simulator
{
/// Replays the ego state recorded in a rosbag during the first `replay_duration` seconds of the
/// scenario, then releases the ego to closed-loop control.
///
/// Time mapping follows the same convention as simple_sensor_simulator's
/// PerceptionReproducerSensor so that a single `replay_start_time` value keeps the ego replay and
/// the perception replay synchronized:
///   scenario_time = (bag reception time - bag metadata starting_time) - replay_start_time
class EgoBagReplayer
{
public:
  struct Sample
  {
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Twist twist;
    geometry_msgs::msg::Accel accel;
  };

  explicit EgoBagReplayer(
    const std::string & bag_path, double replay_start_time, double replay_duration,
    const std::string & odometry_topic = "/localization/kinematic_state",
    const std::string & acceleration_topic = "/localization/acceleration");

  /// Called once per frame before API::updateEntitiesStatusInSim() so that the injected status is
  /// forwarded to the vehicle simulation with the overwrite flag in the very same frame.
  auto update(double current_scenario_time, entity::EntityManager & entity_manager) -> void;

  /// Returns the sample interpolated at the given scenario time, clamped to the recorded range.
  auto interpolate(double scenario_time) const -> Sample;

private:
  struct OdometrySample
  {
    int64_t time_ns;  ///< scenario time [ns]
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Twist twist;
  };

  struct AccelerationSample
  {
    int64_t time_ns;  ///< scenario time [ns]
    geometry_msgs::msg::Accel accel;
  };

  const double replay_duration_;

  bool switched_ = false;

  std::vector<OdometrySample> odometries_;
  std::vector<AccelerationSample> accelerations_;
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__REPLAY__EGO_BAG_REPLAYER_HPP_
