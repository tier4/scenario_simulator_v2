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

#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__POLYLINE_TRAJECTORY_FOLLOWER_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__POLYLINE_TRAJECTORY_FOLLOWER_HPP_

#include <optional>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/polyline_trajectory.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{
struct PolylineTrajectoryFollower
{
public:
  explicit PolylineTrajectoryFollower(
    const traffic_simulator_msgs::msg::EntityStatus & entity_status,
    const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils);

  auto makeUpdatedStatus(
    const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
    const double step_time, const double matching_distance,
    const std::optional<double> target_speed /*= std::nullopt*/) const
    -> std::optional<EntityStatus>;

private:
  const traffic_simulator_msgs::msg::EntityStatus entity_status;
  const traffic_simulator_msgs::msg::BehaviorParameter behavior_parameter;
  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils;

  auto discard_the_front_waypoint_and_recurse(
    const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
    const double step_time, const double matching_distance,
    const std::optional<double> target_speed) const -> std::optional<EntityStatus>;
  // PolylineTrajectoryFollower(const PolylineTrajectoryFollower & other);
  // PolylineTrajectoryFollower & operator=(const PolylineTrajectoryFollower & other);
  // PolylineTrajectoryFollower(PolylineTrajectoryFollower && other) noexcept;
  // PolylineTrajectoryFollower & operator=(PolylineTrajectoryFollower && other) noexcept;
  // ~PolylineTrajectoryFollower();
};
}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__POLYLINE_TRAJECTORY_FOLLOWER_HPP_
