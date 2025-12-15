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

#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__FOLLOW_WAYPOINT_CONTROLLER_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__FOLLOW_WAYPOINT_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>
#include <geometry/quaternion/quaternion_to_euler.hpp>
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

struct PredictedEntityStatus
{
  /// @note compile-time flag to switch between 1D kinematics (false) and lanelet-aware distance (true)
  /// set to true for high-accuracy validation/debugging, default false for production performance
  static constexpr bool use_distance_along_lanelet = false;

  double traveled_distance;
  double travel_time;

  explicit PredictedEntityStatus(const traffic_simulator_msgs::msg::EntityStatus & status)
  : traveled_distance(0.0), travel_time(0.0), entity_status_(status)
  {
  }

  explicit PredictedEntityStatus(
    const traffic_simulator_msgs::msg::EntityStatus & status, const double traveled_distance,
    const double travel_time)
  : traveled_distance(traveled_distance), travel_time(travel_time), entity_status_(status)
  {
  }

  /**
   * @brief Advance simulation by one time step with given acceleration.
   *
   * If use_distance_along_lanelet=false (default): Uses 1D kinematic model with trapezoidal integration.
   * Speed and acceleration updated directly. Distance integrated using average speed (exact for constant acceleration).
   *
   * If use_distance_along_lanelet=true: Uses lanelet-aware distance calculation for high accuracy.
   * Requires update_entity_status and distance_along_lanelet functions.
   *
   * @param step_acceleration Acceleration to apply [m/s²]
   * @param step_time Time step duration [s]
   * @param update_entity_status Function to update entity status (used when use_distance_along_lanelet=true)
   * @param distance_along_lanelet Function to compute distance along lanelet (used when use_distance_along_lanelet=true)
   */
  template <typename UpdateEntityStatusFunc, typename DistanceAlongLaneletFunc>
  auto step(
    const double step_acceleration, const double step_time,
    const UpdateEntityStatusFunc & update_entity_status,
    const DistanceAlongLaneletFunc & distance_along_lanelet) -> void
  {
    const auto current_speed = entity_status_.action_status.twist.linear.x;
    const auto desired_speed = current_speed + step_acceleration * step_time;

    if constexpr (use_distance_along_lanelet) {
      const auto desired_velocity = [&]() {
        const auto euler = math::geometry::convertQuaternionToEulerAngle(entity_status_.pose.orientation);
        /// @note pitch is negated to match the convention: positive pitch = upward motion
        return geometry_msgs::build<geometry_msgs::msg::Vector3>()
          .x(std::cos(-euler.y) * std::cos(euler.z) * desired_speed)
          .y(std::cos(-euler.y) * std::sin(euler.z) * desired_speed)
          .z(std::sin(-euler.y) * desired_speed);
      }();
      const auto position_before_update = entity_status_.pose.position;
      entity_status_ = update_entity_status(entity_status_, desired_velocity);
      traveled_distance += distance_along_lanelet(position_before_update, entity_status_.pose.position);
    } else {
      entity_status_.action_status.twist.linear.x = desired_speed;
      entity_status_.action_status.accel.linear.x = (desired_speed - current_speed) / step_time;
      traveled_distance += (current_speed + desired_speed) * 0.5 * step_time;
    }
    travel_time += step_time;
  }

  auto getEntityStatus() const -> const traffic_simulator_msgs::msg::EntityStatus &
  {
    return entity_status_;
  }
  auto getSpeed() const -> double { return entity_status_.action_status.twist.linear.x; }
  auto getAcceleration() const -> double { return entity_status_.action_status.accel.linear.x; }

  auto isImmobile(const double tolerance) const -> bool
  {
    return std::abs(entity_status_.action_status.twist.linear.x) < tolerance &&
           std::abs(entity_status_.action_status.accel.linear.x) < tolerance;
  }

private:
  traffic_simulator_msgs::msg::EntityStatus entity_status_;
};

class FollowWaypointController
{
  const double step_time;
  const bool with_braking;

  const double max_speed;
  const double max_acceleration;
  const double max_acceleration_rate;
  const double max_deceleration;
  const double max_deceleration_rate;

  const double target_speed;

  /*
     Acceptable time step inaccuracy, allowing the time to be rounded up to the
     full number of steps.

     There is no technical basis for this value, it was determined based on
     Dawid Moszynski experiments.
  */
  static constexpr double step_time_tolerance = 1e-6;

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
  static constexpr std::size_t number_of_acceleration_candidates = 30;

  /*
     This is a debugging method, it is not worth giving it much attention.
  */
  template <typename StreamType>
  friend auto operator<<(StreamType & stream, const FollowWaypointController & c) -> StreamType &
  {
    stream << std::setprecision(16) << std::fixed;
    stream << "FollowWaypointController: step_time: " << c.step_time
           << ", with_braking: " << c.with_braking << ", max_speed: " << c.max_speed
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

  /**
   * @brief Calculate acceleration limits with jerk constraints.
   *
   * Computes the valid range [min_acceleration, max_acceleration] that respects:
   * - Jerk constraints (maximum allowed rate of acceleration change)
   * - Speed must remain within valid range after applying the acceleration
   * - Acceleration bounds (max_acceleration, -max_deceleration)
   *
   * @param acceleration The current acceleration value [m/s²].
   * @param speed The current speed value [m/s].
   * @return std::pair<double, double> [min_acceleration, max_acceleration] valid range [m/s²].
   *
   * @note PRECONDITIONS - This method assumes normal operating conditions:
   * 1. Applying any valid acceleration from the returned range will result in speed >= 0.0
   *    (vehicle will not move backward after this step)
   * 2. Applying any valid acceleration from the returned range will result in speed <= target_speed
   *    (vehicle will not exceed target speed after this step)
   *
   * These preconditions must be satisfied before calling this method.
   * If the vehicle is in an abnormal state where these conditions cannot be met
   * (e.g., even maximum acceleration would result in negative speed, or even maximum
   * braking would result in speed > target_speed), then clampAcceleration() must be
   * called first to handle these edge cases and return corrective acceleration.
   */
  auto getAccelerationLimits(const double acceleration, const double speed) const
    -> std::pair<double, double>;

  auto clampAcceleration(
    const double candidate_acceleration, const double acceleration, const double speed) const
    -> double;

  auto getPredictedStopEntityStatusWithoutConsideringTime(
    const double step_acceleration, const double remaining_distance,
    const traffic_simulator_msgs::msg::EntityStatus & entity_status,
    const std::function<traffic_simulator_msgs::msg::EntityStatus(
      const traffic_simulator_msgs::msg::EntityStatus &, const geometry_msgs::msg::Vector3 &)> &
      update_entity_status,
    const std::function<
      double(const geometry_msgs::msg::Point &, const geometry_msgs::msg::Point &)> &
      distance_along_lanelet) const -> std::optional<PredictedEntityStatus>;

public:
  /*
     Accuracy of the remaining distance to the waypoint at the moment
     of arrival with a specified time.

     There is no technical basis for this value, it was determined based on
     Dawid Moszynski experiments.
  */
  static constexpr double remaining_distance_tolerance = 0.1;

  /*
     Achieving official epsilon (1e-16) accuracy when using doubles is
     difficult for this reason the controller uses less accuracy.

     There is no technical basis for this value, it was determined based on
     Dawid Moszynski experiments.
  */
  static constexpr double local_epsilon = 1e-12;

  explicit constexpr FollowWaypointController(
    const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter,
    const double step_time, const bool with_braking,
    const std::optional<double> & target_speed = std::nullopt)
  : step_time{step_time},
    with_braking{with_braking},
    max_speed{behavior_parameter.dynamic_constraints.max_speed},
    max_acceleration{behavior_parameter.dynamic_constraints.max_acceleration},
    max_acceleration_rate{behavior_parameter.dynamic_constraints.max_acceleration_rate},
    max_deceleration{behavior_parameter.dynamic_constraints.max_deceleration},
    max_deceleration_rate{behavior_parameter.dynamic_constraints.max_deceleration_rate},
    target_speed{(target_speed) ? target_speed.value() : max_speed}
  {
  }

  /**
   * @brief Calculate the optimal acceleration for a single discrete step
   *        to reach a desired target speed while respecting a maximum jerk constraint.
   *
   * This method computes the acceleration to be applied in the current time step
   * so that the final speed after a series of discrete steps equals the target speed,
   * assuming constant jerk (linear change of acceleration) over all steps.
   *
   * @param current_speed The current speed at the start of the step [m/s].
   * @param target_speed  The desired target speed to reach after N discrete steps [m/s].
   * @param max_jerk      The maximum allowed rate of change of acceleration [m/s³].
   * @return The optimal acceleration to apply in the current step [m/s²].
   *
   * @note
   * Algorithm derivation for discrete-time motion with constant jerk:
   *
   * 1. Estimating the number of steps N:
   *
   *    For motion with constant jerk j, the velocity change over time t is:
   *
   *      Δv = a₀·t + 0.5·j·t²
   *
   *    Assuming we start from rest (a₀ = 0) for the jerk phase, the time required is:
   *
   *      t = sqrt(2·|Δv| / j)
   *
   *    Converting to discrete steps with step_time dt:
   *
   *      N = round(t / dt) = round(sqrt(2·|Δv| / j) / dt)
   *
   *    where Δv = target_speed - current_speed, and N ≥ 1.
   *
   * 2. Computing base acceleration:
   *
   *    Without jerk, uniform acceleration to reach Δv over N steps would be:
   *
   *      a_base = Δv / (N·dt)
   *
   * 3. Deriving the jerk correction:
   *
   *    Why we need jerk correction: If we simply used a_base (constant acceleration),
   *    we would ignore the constraint that acceleration must change smoothly with
   *    limited jerk. By applying constant jerk, acceleration increases/decreases
   *    linearly over time, which means we accumulate additional velocity change
   *    beyond what a_base alone would provide. The jerk correction compensates for
   *    this by adjusting the initial acceleration a[0] so that the total velocity
   *    change after N steps exactly matches the desired Δv.
   *
   *    With constant jerk j applied over N steps, acceleration changes linearly:
   *
   *      a[k] = a[0] + j·dt·k,  for k = 0, 1, ..., N-1
   *
   *    Total velocity change is the sum of accelerations times dt:
   *
   *      Δv = dt · Σ(a[k]) = dt · Σ(a[0] + j·dt·k)
   *         = dt · [N·a[0] + j·dt·Σ(k)]
   *         = dt · [N·a[0] + j·dt·(0 + 1 + ... + (N-1))]
   *         = dt · [N·a[0] + j·dt·(N·(N-1)/2)]
   *
   *    Solving for a[0] (the initial acceleration to apply):
   *
   *      a[0] = Δv/(N·dt) - j·dt·(N-1)/2
   *
   *    Breaking this into components:
   *
   *      a[0] = a_base + a_correction
   *
   *    where:
   *      - a_base = Δv/(N·dt)  (uniform acceleration component)
   *      - a_correction = -j·dt·(N-1)/2  (jerk correction)
   *
   *    However, the sign of jerk depends on acceleration/deceleration direction:
   *      - For acceleration (Δv > 0): jerk is positive, correction is positive
   *      - For deceleration (Δv < 0): jerk is negative, correction is negative
   *
   *    Therefore:
   *
   *      a_correction = sign(Δv) · j · dt · (N-1) / 2
   *
   * 4. Final formula:
   *
   *      a_optimal = a_base + a_correction
   *                = Δv/(N·dt) + sign(Δv)·j·dt·(N-1)/2
   *
   * @note
   * - This is a discrete approximation; small errors may occur due to rounding of N.
   * - The method does not clip the acceleration to physical limits, so the caller must
   *   apply additional clamping to ensure the acceleration remains within
   *   max_acceleration and max_deceleration bounds.
   * - The formula is symmetric for acceleration and deceleration - only the sign of
   *   the jerk correction term changes based on the velocity difference direction.
   */

  auto accelerationWithJerkConstraint(
    const double current_speed, const double target_speed, const double acceleration_rate) const
    -> double;

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
            [](auto && vertex) { return std::isfinite(vertex.time); });
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
    const traffic_simulator_msgs::msg::EntityStatus & entity_status,
    const std::function<traffic_simulator_msgs::msg::EntityStatus(
      const traffic_simulator_msgs::msg::EntityStatus &, const geometry_msgs::msg::Vector3 &)> &
      update_entity_status,
    const std::function<
      double(const geometry_msgs::msg::Point &, const geometry_msgs::msg::Point &)> &
      distance_along_lanelet) const -> std::optional<PredictedEntityStatus>;
  /*
     This allows the best acceleration to be found for the current conditions,
     without taking into account the arrival time - this is the case when every
     next point of the trajectory has no specified time.
  */
  auto getAcceleration(
    const double remaining_distance,
    const traffic_simulator_msgs::msg::EntityStatus & entity_status,
    const std::function<traffic_simulator_msgs::msg::EntityStatus(
      const traffic_simulator_msgs::msg::EntityStatus &, const geometry_msgs::msg::Vector3 &)> &
      update_entity_status,
    const std::function<
      double(const geometry_msgs::msg::Point &, const geometry_msgs::msg::Point &)> &
      distance_along_lanelet) const -> double;

  /*
     This allows the best acceleration to be found for the current conditions,
     taking into account the arrival time.
  */
  auto getAcceleration(
    const double remaining_time_source, const double remaining_distance,
    const traffic_simulator_msgs::msg::EntityStatus & entity_status,
    const std::function<traffic_simulator_msgs::msg::EntityStatus(
      const traffic_simulator_msgs::msg::EntityStatus &, const geometry_msgs::msg::Vector3 &)> &
      update_entity_status,
    const std::function<
      double(const geometry_msgs::msg::Point &, const geometry_msgs::msg::Point &)> &
      distance_along_lanelet) const -> double;

  auto areConditionsOfArrivalMet(
    const double acceleration, const double speed, const double distance) const -> double
  {
    return (!with_braking || std::abs(speed) < local_epsilon) &&
           std::abs(acceleration) < local_epsilon && distance < remaining_distance_tolerance;
  }
};
}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__FOLLOW_WAYPOINT_CONTROLLER_HPP_
