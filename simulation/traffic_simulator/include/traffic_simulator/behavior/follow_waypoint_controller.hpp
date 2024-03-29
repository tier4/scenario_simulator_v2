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

  auto moveStraight(const double step_acceleration, const double step_time) -> void
  {
    acceleration = step_acceleration;
    speed += acceleration * step_time;
    traveled_distance += speed * step_time;
    travel_time += step_time;
  }

  auto isImmobile(const double tolerance) const
  {
    return std::abs(speed) < tolerance && std::abs(acceleration) < tolerance;
  }

  template <typename StreamType>
  friend auto operator<<(StreamType & stream, const PredictedState & state) -> StreamType &
  {
    stream << std::setprecision(16) << std::fixed;
    stream << "PredictedState: acceleration: " << state.acceleration << ", speed: " << state.speed
           << ", distance: " << state.traveled_distance << ", time: " << state.travel_time << ". ";
    return stream;
  }
};

class FollowWaypointController
{
  const double step_time;
  const bool with_breaking;

  const double max_speed;
  const double max_acceleration;
  const double max_acceleration_rate;
  const double max_deceleration;
  const double max_deceleration_rate;

  const double target_speed;

  /*
     Achieving official epsilon (1e-16) accuracy when using doubles is
     difficult for this reason the controller uses less accuracy.

     There is no technical basis for this value, it was determined based on
     Dawid Moszynski experiments.
  */
  static constexpr double local_epsilon = 1e-12;

  /*
     Acceptable time step inaccuracy, allowing the time to be rounded up to the
     full number of steps.

     There is no technical basis for this value, it was determined based on
     Dawid Moszynski experiments.
  */
  static constexpr double step_time_tolerance = 1e-6;

  /*
     Accuracy of the final arrival distance at a waypoint with a specified
     time.

     There is no technical basis for this value, it was determined based on
     Dawid Moszynski experiments.
  */
  static constexpr double finish_distance_tolerance = 1e-4;

  /*
     Accuracy of the predicted arrival distance at the waypoint with the
     specified time it is only used to detect in advance that it is most likely
     impossible to arrive at a sufficient final accuracy.

     There is no technical basis for this value, it was determined based on
     Dawid Moszynski experiments.
  */
  static constexpr double predicted_distance_tolerance = 1.0;

  /*
     Number of considered acceleration candidates. This is a discretization of
     the current range of [min_acceleration, max_acceleration].

     There is no technical basis for this value, it was determined based on
     Dawid Moszynski experiments.
  */
  static constexpr std::size_t number_of_acceleration_candidates = 20;

  /*
     This is a debugging method, it is not worth giving it much attention.
  */
  template <typename StreamType>
  friend auto operator<<(StreamType & stream, const FollowWaypointController & c) -> StreamType &
  {
    stream << std::setprecision(16) << std::fixed;
    stream << "FollowWaypointController: step_time: " << c.step_time
           << ", with_breaking: " << c.with_breaking << ", max_speed: " << c.max_speed
           << ", max_acceleration: " << c.max_acceleration
           << ", max_deceleration: " << c.max_deceleration
           << ", max_acceleration_rate: " << c.max_acceleration_rate
           << ", max_deceleration_rate: " << c.max_deceleration_rate
           << ", target_speed: " << c.target_speed << ". ";
    return stream;
  }

  /*
     This provides an analytical method for calculating the acceleration that
     will allow the best possible accuracy in arriving at the waypoint during
     the last few frames. It provides the calculation of acceleration for
     arriving at waypoint without and with braking.

     The method allows to calculate only the last 3 steps without braking and 4
     with braking, because analytical calculation of a greater number of steps
     is difficult - a greater number of steps is processed through the
     prediction of the controller.

     Details:

     - Acceleration at the last step equal to 0.0, therefore in the penultimate
       step the acceleration is set to 0.0

     - Only with braking: speed at the last and penultimate step equal to 0.0,
       therefore when there are 3 steps remain the acceleration is calculated
       in such a way as to set the speed to 0.0 (a=-v/t)

     - Only with braking: because the speed is set to 0.0 when there are 3
       steps remain, in the last 3 steps no distance will be traveled,
       therefore, when there are 4 steps remain, it is necessary to choose the
       acceleration in such a way as to travel exactly the remaining distance
       in this step (s = at^2/2+vt -> a = 2(s-vt)/t^2).

     - Only without braking: in the penultimate step, the acceleration is set
       to 0, therefore, when 3 steps remain it is necessary to choose the
       acceleration in such a way as to ensure that in the penultimate and last
       step the distance traveled is equal to the remaining distance (s = last
       + penultimate = (v+at)t + at^2/2+vt -> a = 2/3(s-2vt)/t^2).
  */
  auto getAnalyticalAccelerationForLastSteps(
    const double remaining_time, const double remaining_distance, const double acceleration,
    const double speed) const -> double;

  /*
     This allows the correct counting  of the remaining number of steps.
     Without this, inaccuracy sometimes results in a decrease of more than 1
     step which can cause major difficulties if we want to arrive at the
     waypoint at the precise time.
  */
  auto roundTimeToFullStepsWithTolerance(
    const double remaining_time_source, const double time_tolerance) const -> double;

  /*
     This allows to calculate how much time (rounded up to whole steps) is
     needed to reach acceleration equal to zero - which is necessary in case of
     achieving the maximum speed (to not exceed it) and achieving the speed
     equal to 0.0 (to not start moving backwards).
  */
  auto getTimeRequiredForNonAcceleration(const double acceleration) const -> double;

  /*
     This allows the calculation of acceleration limits that meet the
     constraints. The limits depend on the current acceleration, the current
     speed and the limits on the change in acceleration (rate).
  */
  auto getAccelerationLimits(const double acceleration, const double speed) const
    -> std::pair<double, double>;

  auto clampAcceleration(
    const double candidate_acceleration, const double acceleration, const double speed) const
    -> double;

  auto moveStraight(PredictedState & state, const double candidate_acceleration) const -> void;

  auto getPredictedStopStateWithoutConsideringTime(
    const double step_acceleration, const double remaining_distance, const double acceleration,
    const double speed) const -> std::optional<PredictedState>;

public:
  explicit constexpr FollowWaypointController(
    const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter,
    const double step_time, const bool with_breaking,
    const std::optional<double> & target_speed = std::nullopt)
  : step_time{step_time},
    with_breaking{with_breaking},
    max_speed{behavior_parameter.dynamic_constraints.max_speed},
    max_acceleration{behavior_parameter.dynamic_constraints.max_acceleration},
    max_acceleration_rate{behavior_parameter.dynamic_constraints.max_acceleration_rate},
    max_deceleration{behavior_parameter.dynamic_constraints.max_deceleration},
    max_deceleration_rate{behavior_parameter.dynamic_constraints.max_deceleration_rate},
    target_speed{(target_speed) ? target_speed.value() : max_speed}
  {
  }

  /*
     This is a debugging method, it is not worth giving it much attention.
  */
  auto getFollowedWaypointDetails(
    const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory) const
    -> std::string
  {
    if (const auto & vertices = polyline_trajectory.shape.vertices; !vertices.empty()) {
      std::stringstream waypoint_details;
      waypoint_details << "Currently followed waypoint: ";
      if (const auto first_waypoint_with_arrival_time_specified = std::find_if(
            vertices.begin(), vertices.end(),
            [](auto && vertex) { return not std::isnan(vertex.time); });
          first_waypoint_with_arrival_time_specified !=
          std::end(polyline_trajectory.shape.vertices)) {
        waypoint_details << "[" << first_waypoint_with_arrival_time_specified->position.position.x
                         << ", " << first_waypoint_with_arrival_time_specified->position.position.y
                         << "] with specified time equal to "
                         << first_waypoint_with_arrival_time_specified->time << "s. ";
      } else {
        waypoint_details << "[" << vertices.back().position.position.x << ", "
                         << vertices.back().position.position.y << "] without specified time. ";
      }
      return waypoint_details.str();
    } else {
      return "No followed waypoint - trajectory is empty.";
    }
  }

  /*
     This allows to predict the state that will be reached if the current
     acceleration is changed from acceleration to step_acceleration.
  */
  auto getPredictedWaypointArrivalState(
    const double step_acceleration, const double remaining_time, const double remaining_distance,
    const double acceleration, const double speed) const -> std::optional<PredictedState>;
  /*
     This allows the best acceleration to be found for the current conditions,
     without taking into account the arrival time - this is the case when every
     next point of the trajectory has no specified time.
  */
  auto getAcceleration(
    const double remaining_distance, const double acceleration, const double speed) const -> double;

  /*
     This allows the best acceleration to be found for the current conditions,
     taking into account the arrival time.
  */
  auto getAcceleration(
    const double remaining_time_source, const double remaining_distance, const double acceleration,
    const double speed) const -> double;

  auto areConditionsOfArrivalMet(
    const double acceleration, const double speed, const double distance) const -> double
  {
    return (!with_breaking || std::abs(speed) < local_epsilon) &&
           std::abs(acceleration) < local_epsilon && distance < finish_distance_tolerance;
  }
};
}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__FOLLOW_WAYPOINT_CONTROLLER_HPP_
