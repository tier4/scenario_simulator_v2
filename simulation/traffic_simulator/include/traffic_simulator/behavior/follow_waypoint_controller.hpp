// Copyright 2023 TIER IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__FOLLOW_WAYPOINT_CONTROLLER_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__FOLLOW_WAYPOINT_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/polyline_trajectory.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{
struct ControllerError : public common::Error
{
  template <typename... Ts>
  explicit ControllerError(Ts &&... xs)
  : common::Error(common::concatenate(
      "An error occurred in the internal controller operation in the FollowTrajectoryAction. ",
      "Please report the following information to the developer: ",
      std::forward<decltype(xs)>(xs)...))
  {
  }
};

struct PredictedState
{
  double acceleration;
  double speed;
  double traveled_distance;
  double travel_time;

  auto straightMove(const double acc, const double step_time) -> void
  {
    acceleration = acc;
    speed += acceleration * step_time;
    traveled_distance += speed * step_time;
    travel_time += step_time;
  }

  template <typename StreamType>
  friend auto operator<<(StreamType & stream, const PredictedState & state) -> StreamType &
  {
    stream << std::setprecision(16) << std::fixed;
    stream << "acceleration: " << state.acceleration << ", speed: " << state.speed
           << ", distance: " << state.traveled_distance << ", time: " << state.travel_time;
    return stream;
  }
};

class FollowWaypointController
{
  const double step_time;
  const bool with_breaking;
  // constraints
  double max_speed;
  double max_acceleration;
  double max_acceleration_rate;
  double max_deceleration;
  double max_deceleration_rate;
  // accuracy
  const double local_epsilon{1e-12};
  const double step_time_tolerance{1e-6};
  const double finish_distance_tolerance{1e-5};
  const double predicted_distance_tolerance{0.5};
  const int number_of_acceleration_candidates{20};

  template <typename StreamType>
  friend auto operator<<(StreamType & stream, const FollowWaypointController & c) -> StreamType &
  {
    stream << std::setprecision(16) << std::fixed;
    stream << "step_time: " << c.step_time << ", with_breaking: " << c.with_breaking
           << ", max_speed: " << c.max_speed << ", max_acceleration: " << c.max_acceleration
           << ", max_deceleration: " << c.max_deceleration
           << ", max_acceleration_rate: " << c.max_acceleration_rate
           << ", max_deceleration_rate: " << c.max_deceleration_rate << ".";
    return stream;
  }

  auto getAnalyticalAccelerationForLastSteps(
    const double remaining_time, const double left_distance, const double acceleration,
    const double speed) const -> double;
  auto roundTimeToFullStepsWithTolerance(
    const double remaining_time_source, double time_tolerance) const -> double;
  auto timeRequiredForNonAcceleration(const double acceleration) const -> double;
  auto getAccelerationLimits(const double acceleration, const double speed) const
    -> std::pair<double, double>;
  auto clampAcceleration(
    const double candidate_acc, const double acceleration, const double speed) const -> double;
  auto straightMove(PredictedState & state, const double candidate_acc) const -> void;

public:
  FollowWaypointController(
    const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter, double step_time,
    bool with_breaking)
  : step_time{step_time},
    with_breaking{with_breaking},
    max_speed{behavior_parameter.dynamic_constraints.max_speed},
    max_acceleration{behavior_parameter.dynamic_constraints.max_acceleration},
    max_acceleration_rate{behavior_parameter.dynamic_constraints.max_acceleration_rate},
    max_deceleration{behavior_parameter.dynamic_constraints.max_deceleration},
    max_deceleration_rate{behavior_parameter.dynamic_constraints.max_deceleration_rate}
  {
  }

  auto distanceMeetsAccuracyRestrictions(const double distance) const -> double
  {
    return distance < finish_distance_tolerance;
  }
  auto getPredictedWaypointArrivalState(
    const double step_acc, const double remaining_time, const double left_distance,
    const double acceleration, const double speed) const -> std::optional<PredictedState>;
  auto getAcceleration(
    const double remaining_time_source, const double left_distance, const double acceleration,
    const double speed) const -> double;
};

}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__FOLLOW_WAYPOINT_CONTROLLER_HPP_
