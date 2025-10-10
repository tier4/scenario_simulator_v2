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

#include <traffic_simulator/behavior/follow_waypoint_controller.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{
auto FollowWaypointController::clampAcceleration(
  const double candidate_acceleration, const double acceleration, const double speed) const
  -> double
{
  /*
     If even maximum jerk-constrained acceleration would still result in negative speed after this step,
     the vehicle is in backward motion state. Return maximum braking to return to positive speed.
  */
  if (speed + (acceleration + max_acceleration_rate * step_time) * step_time < 0.0) {
    return std::min(
      accelerationWithJerkConstraint(speed, 0.0, max_acceleration_rate), max_acceleration);
    /*
     If even minimum jerk-constrained acceleration (maximum braking) would still result in speed exceeding
     target_speed after this step, the vehicle is moving too fast. Return maximum braking to reduce speed.
  */
  } else if (
    speed + (acceleration - max_deceleration_rate * step_time) * step_time > target_speed) {
    return std::max(
      accelerationWithJerkConstraint(speed, target_speed, max_deceleration_rate),
      -max_deceleration);
  } else {
    auto [local_min_acceleration, local_max_acceleration] =
      getAccelerationLimits(acceleration, speed);
    return std::clamp(candidate_acceleration, local_min_acceleration, local_max_acceleration);
  }
}

auto FollowWaypointController::getAnalyticalAccelerationForLastSteps(
  const double remaining_time, const double remaining_distance, const double acceleration,
  const double speed) const -> double
{
  if (remaining_time <= step_time * 2.0) {
    if (!with_breaking || (std::abs(speed) < local_epsilon)) {
      // Step in which the acceleration is set to 0.0.
      return clampAcceleration(0.0, acceleration, speed);
    } else {
      // If it is the penultimate step with braking, speed should be equal to 0.0.
      throw ControllerError(
        "The speed in the penultimate step for last waypoint is different from zero, speed: ",
        speed, ", acceleration: ", acceleration, ", remaining_time: ", remaining_time,
        ", remaining_distance: ", remaining_distance, ". ", *this);
    }
  } else if (remaining_time <= step_time * 3.0) {
    if (with_breaking) {
      // Step in which the speed is set to 0.0.
      return clampAcceleration(-speed / step_time, acceleration, speed);
    } else {
      /*
         Step in which acceleration is set to ensure that the remaining
         distance is traveled in next 2 steps.
      */
      const double numerator = 2.0 / 3.0 * (remaining_distance - 2.0 * speed * step_time);
      return clampAcceleration(numerator / std::pow(step_time, 2), acceleration, speed);
    }
  } else if (remaining_time <= step_time * 4.0) {
    if (with_breaking) {
      /*
         Step in which acceleration is set to ensure that the remaining
         distance is traveled in next step.
      */
      const double numerator = 2.0 * (remaining_distance - speed * step_time);
      return clampAcceleration(numerator / std::pow(step_time, 2), acceleration, speed);
    }
  }

  throw common::SemanticError(
    "An analytical calculation of acceleration is not available for a time step greater than 4 (",
    step_time * 4, "s) in the case of braking and greater than 3 (", step_time * 3,
    "s) without braking.");
}

auto FollowWaypointController::roundTimeToFullStepsWithTolerance(
  const double remaining_time_source, const double time_tolerance) const -> double
{
  if (std::isnan(remaining_time_source) || std::isinf(remaining_time_source)) {
    throw ControllerError(
      "Value remaining_time_source (", remaining_time_source,
      ") is NaN or inf - it cannot be rounded.");
  } else if (
    remaining_time_source >=
    static_cast<double>(std::numeric_limits<std::size_t>::max()) * step_time) {
    throw ControllerError(
      "Exceeded the range of the variable type <std::size_t> (", remaining_time_source, "/",
      step_time, ") - the value is too large. Probably the distance to the waypoint is extremely",
      " large and the predicted time exceeds the range.");
  } else {
    const auto number_of_steps_source = static_cast<std::size_t>(remaining_time_source / step_time);
    const double remaining_time = number_of_steps_source * step_time;
    if (const double time_difference =
          (remaining_time_source / step_time) - (remaining_time / step_time);
        time_difference > 1.0 - time_tolerance) {
      return remaining_time + step_time + time_tolerance;
    } else {
      return remaining_time + time_tolerance;
    }
  }
}

auto FollowWaypointController::getTimeRequiredForNonAcceleration(const double acceleration) const
  -> double
{
  const double acceleration_rate =
    (acceleration > 0.0) ? max_deceleration_rate : max_acceleration_rate;
  return std::ceil(std::abs(acceleration) / (acceleration_rate * step_time)) * step_time;
}

auto FollowWaypointController::accelerationWithJerkConstraint(
  const double current_speed, const double target_speed, const double acceleration_rate) const
  -> double
{
  const auto delta_speed = target_speed - current_speed;

  /// @note use absolute value of acceleration_rate to ensure it is always positive
  const auto jerk_magnitude = std::abs(acceleration_rate);

  /// @note minimal number of discrete steps required to reach target_speed based on maximum jerk and time step
  const auto minimum_discrete_steps =
    std::max(1.0, std::round(std::sqrt(2 * std::abs(delta_speed) / jerk_magnitude) / step_time));

  /// @note acceleration (without jerk correction) required to reach target_speed assuming constant acceleration
  const auto base_acceleration = delta_speed / (minimum_discrete_steps * step_time);

  /// @note sign of the jerk correction based on whether we are accelerating or decelerating
  const auto sign = (delta_speed >= 0.0) ? 1.0 : -1.0;

  /// @note correction term due to linear change of acceleration (jerk),
  /// this ensures that after all discrete steps, the final speed matches target_speed
  const auto jerk_correction =
    sign * jerk_magnitude * step_time * (minimum_discrete_steps - 1.0) / 2.0;

  return base_acceleration + jerk_correction;
}

auto FollowWaypointController::getAccelerationLimits(
  const double acceleration, const double speed) const -> std::pair<double, double>
{
  /// @note see header file for detailed preconditions and assumptions
  const auto min_acceleration_zero_speed =
    accelerationWithJerkConstraint(speed, 0.0, max_deceleration_rate);
  const auto max_acceleration_target_speed =
    accelerationWithJerkConstraint(speed, target_speed, max_acceleration_rate);

  const auto local_min_acceleration = std::max(
    std::max(min_acceleration_zero_speed, -max_deceleration),
    acceleration - max_deceleration_rate * step_time);
  const auto local_max_acceleration = std::min(
    std::min(max_acceleration_target_speed, max_acceleration),
    acceleration + max_acceleration_rate * step_time);

  /*
      It can occur when local_min_acceleration is equal to local_max_acceleration, leaving no choice.

      This happens when the vehicle must reach both zero speed and zero acceleration simultaneously.
      For example: with current acceleration = -5.0 m/s² and low speed that keeps decreasing,
      entity must increase acceleration in the next steps to reach speed equal to 0.0 and acceleration = 0.0
      at the same moment.

      In such cases, local_min_acceleration (needed to reach acceleration equal to 0.0 at speed 0.0) can be equal to local_max_acceleration
      (the maximum increase allowed by max_acceleration_rate constraint).
  */
  if (local_max_acceleration + local_epsilon < local_min_acceleration - local_epsilon) {
    throw ControllerError(
      "Contradictory acceleration limits: local_max_acceleration (", local_max_acceleration,
      ") < local_min_acceleration (", local_min_acceleration,
      ") => difference: ", (local_max_acceleration - local_min_acceleration),
      " for acceleration: ", acceleration, " and speed: ", speed, ". Target speed: ", target_speed,
      ". ", *this);
  }

  const double min_speed = speed + local_min_acceleration * step_time;
  const double max_speed = speed + local_max_acceleration * step_time;
  if (
    max_speed < -local_epsilon || max_speed > std::min(max_speed, target_speed) + local_epsilon ||
    min_speed < -local_epsilon || min_speed > std::min(max_speed, target_speed) + local_epsilon) {
    throw ControllerError(
      "Incorrect acceleration limits [", local_min_acceleration, ", ", local_max_acceleration,
      "] for acceleration: ", acceleration, " and speed: ", speed, " -> min_speed: ", min_speed,
      " max_speed: ", max_speed, ". ", *this);
  }

  return {local_min_acceleration, local_max_acceleration};
}

auto FollowWaypointController::getPredictedStopEntityStatusWithoutConsideringTime(
  const double step_acceleration, const double remaining_distance,
  const traffic_simulator_msgs::msg::EntityStatus & entity_status,
  const std::function<traffic_simulator_msgs::msg::EntityStatus(
    const traffic_simulator_msgs::msg::EntityStatus &, const geometry_msgs::msg::Vector3 &)> &
    update_entity_status,
  const std::function<double(
    const geometry_msgs::msg::Point &, const geometry_msgs::msg::Point &)> & distance_along_lanelet)
  const -> std::optional<PredictedEntityStatus>
{
  PredictedEntityStatus predicted_status(entity_status);
  predicted_status.step(step_acceleration, step_time, update_entity_status, distance_along_lanelet);
  while (!predicted_status.isImmobile(local_epsilon)) {
    if (predicted_status.traveled_distance > remaining_distance + predicted_distance_tolerance) {
      return std::nullopt;
    }
    const auto acceleration = predicted_status.getAcceleration();
    const auto speed = predicted_status.getSpeed();
    /// @note clampAcceleration handles edge cases (negative speed or speed > target_speed) by returning corrective acceleration
    if (const auto clamped = clampAcceleration(acceleration, acceleration, speed);
        clamped != acceleration) {
      predicted_status.step(clamped, step_time, update_entity_status, distance_along_lanelet);
    } else {
      auto [local_min_acceleration, local_max_acceleration] =
        getAccelerationLimits(acceleration, speed);
      predicted_status.step(
        local_min_acceleration, step_time, update_entity_status, distance_along_lanelet);
    }
  }
  return predicted_status;
}

auto FollowWaypointController::getPredictedWaypointArrivalState(
  const double step_acceleration, const double remaining_time, const double remaining_distance,
  const traffic_simulator_msgs::msg::EntityStatus & entity_status,
  const std::function<traffic_simulator_msgs::msg::EntityStatus(
    const traffic_simulator_msgs::msg::EntityStatus &, const geometry_msgs::msg::Vector3 &)> &
    update_entity_status,
  const std::function<double(
    const geometry_msgs::msg::Point &, const geometry_msgs::msg::Point &)> & distance_along_lanelet)
  const -> std::optional<PredictedEntityStatus>
{
  const auto brakeUntilImmobility = [&](PredictedEntityStatus & predicted_status) -> auto {
    while (!predicted_status.isImmobile(local_epsilon)) {
      if (predicted_status.travel_time >= remaining_time) {
        return false;
      }
      const auto acceleration = predicted_status.getAcceleration();
      const auto speed = predicted_status.getSpeed();
      /// @note clampAcceleration handles edge cases (negative speed or speed > target_speed) by returning corrective acceleration.
      if (const auto clamped = clampAcceleration(acceleration, acceleration, speed);
          clamped != acceleration) {
        predicted_status.step(clamped, step_time, update_entity_status, distance_along_lanelet);
      } else {
        auto [local_min_acceleration, local_max_acceleration] =
          getAccelerationLimits(acceleration, speed);
        predicted_status.step(
          local_min_acceleration, step_time, update_entity_status, distance_along_lanelet);
      }
    }
    return true;
  };

  /// @todo rename state -> predicted_status
  PredictedEntityStatus state(entity_status);
  if (remaining_time < step_time) {
    return state;
  } else {
    // First step with acceleration equal to step_acceleration.
    state.step(
      clampAcceleration(step_acceleration, state.getAcceleration(), state.getSpeed()), step_time,
      update_entity_status, distance_along_lanelet);

    if (with_breaking) {
      // Predict the current (before acceleration zeroing) braking time required for stopping.
      PredictedEntityStatus breaking_check = state;
      if (!brakeUntilImmobility(breaking_check)) {
        // If complete immobility is not possible - ignore this candidate.
        return std::nullopt;
      } else if (std::abs(breaking_check.travel_time - remaining_time) <= step_time) {
        // If it is breaking time - consider this candidate.
        return breaking_check;
      }
    }

    // If it is not braking time, more time left for driving with constant speed.
    while (std::abs(state.getAcceleration()) > 0.0) {
      if (state.travel_time >= remaining_time) {
        throw ControllerError(
          "It is not the braking time, but there is no time to achieve acceleration equal "
          "to 0.0 - the trajectory does not meet the constraint of having an acceleration "
          "equal to 0.0 on arrival at the followed waypoint, speed: ",
          state.getSpeed(), ", acceleration: ", state.getAcceleration(),
          ", remaining_time: ", remaining_time, ", remaining_distance: ", remaining_distance,
          " step_acceleration: ", step_acceleration, ". ", *this);
      } else {
        state.step(
          clampAcceleration(0.0, state.getAcceleration(), state.getSpeed()), step_time,
          update_entity_status, distance_along_lanelet);
      }
    }

    if (std::abs(state.getSpeed()) <= local_epsilon) {
      // If the previous steps caused the constant speed to be extremely low - ignore this.
      return std::nullopt;
    } else {
      const double const_speed_value = state.getSpeed();

      if (with_breaking) {
        // Predict the current (after acceleration zeroing) braking time required for stopping.
        if (!brakeUntilImmobility(state)) {
          // If complete immobility is not possible - ignore this candidate.
          return std::nullopt;
        } else if (std::abs(state.travel_time - remaining_time) <= step_time) {
          // If it is breaking time - consider this candidate.
          return state;
        }
      }

      // Count the distance and time of movement with constant speed, use this to prediction.
      if (const double const_speed_distance = remaining_distance - state.traveled_distance;
          const_speed_distance >= std::numeric_limits<double>::max() * const_speed_value) {
        throw ControllerError(
          "Exceeded the range of the variable type <double> (", const_speed_distance, "/",
          const_speed_value, ") - the value is too large. Probably the distance",
          " to the waypoint is extremely large and the predicted time exceeds the range.");
      } else {
        const double const_speed_time = const_speed_distance / const_speed_value;

        // Round distance using a time equal to the full number of steps.
        const double rounded_const_speed_time =
          roundTimeToFullStepsWithTolerance(const_speed_time, step_time_tolerance);
        const double rounded_const_speed_distance = rounded_const_speed_time * const_speed_value;
        return PredictedEntityStatus{
          state.getEntityStatus(), rounded_const_speed_distance + state.traveled_distance,
          rounded_const_speed_time + state.travel_time};
      }
    }
  }
}

auto FollowWaypointController::getAcceleration(
  const double remaining_distance, const traffic_simulator_msgs::msg::EntityStatus & entity_status,
  const std::function<traffic_simulator_msgs::msg::EntityStatus(
    const traffic_simulator_msgs::msg::EntityStatus &, const geometry_msgs::msg::Vector3 &)> &
    update_entity_status,
  const std::function<double(
    const geometry_msgs::msg::Point &, const geometry_msgs::msg::Point &)> & distance_along_lanelet)
  const -> double
{
  const auto acceleration = entity_status.action_status.accel.linear.x;
  const auto speed = entity_status.action_status.twist.linear.x;
  const auto [local_min_acceleration, local_max_acceleration] =
    getAccelerationLimits(acceleration, speed);

  const double step_acceleration =
    (local_max_acceleration - local_min_acceleration) / number_of_acceleration_candidates;

  auto min_distance_diff = std::numeric_limits<double>::lowest();

  std::optional<double> best_acceleration = std::nullopt;

  for (std::size_t i = 0; i <= number_of_acceleration_candidates; ++i) {
    const double candidate_acceleration = local_min_acceleration + i * step_acceleration;
    if (const auto predicted_state_opt = getPredictedStopEntityStatusWithoutConsideringTime(
          candidate_acceleration, remaining_distance, entity_status, update_entity_status,
          distance_along_lanelet);
        predicted_state_opt) {
      const auto distance_diff = remaining_distance - predicted_state_opt->traveled_distance;

      /// @note always prefer the smallest positive distance_diff (candidate stops before target)
      /// only accept negative distance_diff (overshooting) if no positive option exists
      const bool current_candidate_stops_before_target = distance_diff >= 0;
      const bool best_candidate_stops_before_target = min_distance_diff >= 0;

      bool should_update_best_candidate = false;
      if (current_candidate_stops_before_target && best_candidate_stops_before_target) {
        /// @note both stop before target - choose the one that stops closer to target
        should_update_best_candidate = (distance_diff < min_distance_diff);
      } else if (current_candidate_stops_before_target && !best_candidate_stops_before_target) {
        /// @note current stops before target, best overshoots - always prefer stopping before
        should_update_best_candidate = true;
      } else if (!current_candidate_stops_before_target && !best_candidate_stops_before_target) {
        /// @note both overshoot target - choose the one that overshoots less
        should_update_best_candidate = (std::abs(distance_diff) < std::abs(min_distance_diff));
      }
      /// @note else: current overshoots but best stops before target - keep best (should_update_best_candidate = false)

      if (should_update_best_candidate) {
        min_distance_diff = distance_diff;
        best_acceleration = candidate_acceleration;
      }
    }
  }

  if (best_acceleration.has_value()) {
    return best_acceleration.value();
  } else {
    throw ControllerError(
      "No acceleration found, speed: ", speed, ", acceleration: ", acceleration,
      ", remaining_time: no specified, remaining_distance: ", remaining_distance, ". ", *this);
  }
}

auto FollowWaypointController::getAcceleration(
  const double remaining_time_source, const double remaining_distance,
  const traffic_simulator_msgs::msg::EntityStatus & entity_status,
  const std::function<traffic_simulator_msgs::msg::EntityStatus(
    const traffic_simulator_msgs::msg::EntityStatus &, const geometry_msgs::msg::Vector3 &)> &
    update_entity_status,
  const std::function<double(
    const geometry_msgs::msg::Point &, const geometry_msgs::msg::Point &)> & distance_along_lanelet)
  const -> double
{
  const auto acceleration = entity_status.action_status.accel.linear.x;
  const auto speed = entity_status.action_status.twist.linear.x;

  /*
     Check if vehicle is in abnormal state (negative speed or speed exceeding target_speed).
     clampAcceleration handles two edge cases:
     (1) Negative speed: vehicle moving backward, apply maximum braking to return to positive speed
     (2) Speed > target_speed: vehicle too fast, apply maximum braking to reduce speed
     If either condition is detected, immediately return the corrective acceleration.
     This prevents the algorithm from attempting normal waypoint-tracking calculations in unsupported states.
  */
  if (const auto clamp_acceleration = clampAcceleration(acceleration, acceleration, speed);
      clamp_acceleration != acceleration) {
    return clamp_acceleration;
  }

  const auto [local_min_acceleration, local_max_acceleration] =
    getAccelerationLimits(acceleration, speed);

  if ((speed + local_min_acceleration * step_time * 0.5) * step_time > remaining_distance) {
    /*
       If in the current conditions, the waypoint will be reached in this step
       even with the lowest possible acceleration set, this prevents cases in
       which the controller is started extremely close to waypoint and nothing
       can be done.
    */
    return local_min_acceleration;
  } else if (std::abs(local_min_acceleration - local_max_acceleration) < local_epsilon) {
    /*
       If the range is so tight that there is no choice.
    */
    return local_min_acceleration;
  } else if (std::isinf(remaining_time_source)) {
    /*
       If remaining time is no specified - this is the case when every next
       point of the trajectory has no specified time.
    */
    return getAcceleration(
      remaining_distance, entity_status, update_entity_status, distance_along_lanelet);
  } else {
    const auto remaining_time =
      roundTimeToFullStepsWithTolerance(remaining_time_source, step_time_tolerance);

    /*
       If last steps, increase accuracy by using analytical calculations.
    */
    if (with_breaking && remaining_time <= step_time * 4) {
      return getAnalyticalAccelerationForLastSteps(
        remaining_time, remaining_distance, acceleration, speed);
    } else if (remaining_time <= step_time * 3) {
      return getAnalyticalAccelerationForLastSteps(
        remaining_time, remaining_distance, acceleration, speed);
    }

    // Create a lambda for candidate compare
    auto min_time_diff = std::numeric_limits<double>::max();
    auto min_distance_diff = std::numeric_limits<double>::max();
    const auto isBetterCandidate = [local_epsilon = this->local_epsilon, &min_time_diff,
                                    &min_distance_diff](
                                     double time_diff, double distance_diff) -> bool {
      const bool same_time = std::abs(time_diff - min_time_diff) < local_epsilon;
      const bool time_better = !same_time && (std::abs(time_diff) < std::abs(min_time_diff));
      const bool distance_better =
        distance_diff >= 0 &&
        (std::abs(distance_diff) < std::abs(min_distance_diff) || min_distance_diff < 0);
      return time_better || (same_time && distance_better);
    };

    // Create a lambda for candidate selection
    std::optional<double> best_acceleration = std::nullopt;
    const auto considerCandidate = [&](double candidate_acceleration) -> void {
      if (
        const auto predicted_state_opt = getPredictedWaypointArrivalState(
          candidate_acceleration, remaining_time, remaining_distance, entity_status,
          update_entity_status, distance_along_lanelet)) {
        const auto time_diff = remaining_time - predicted_state_opt->travel_time;
        const auto distance_diff = remaining_distance - predicted_state_opt->traveled_distance;
        if (isBetterCandidate(time_diff, distance_diff)) {
          min_time_diff = time_diff;
          min_distance_diff = distance_diff;
          best_acceleration = candidate_acceleration;
        }
      }
    };

    // Consider the borderline values and precise value of 0
    considerCandidate(local_min_acceleration);
    if (local_min_acceleration < 0.0 && local_max_acceleration > 0.0) {
      considerCandidate(0.0);
    }
    considerCandidate(local_max_acceleration);

    /// @todo
    /*
       Find the best acceleration for followed waypoint with a specified time.

       It is likely that this search (in the intermediate values) can be
       skipped if condition (min_time_diff < -step_time || min_time_diff >
       step_time) is met however, at the moment this change affects other
       behaviors - this requires further investigation.
    */
    if (const double step_acceleration =
          (local_max_acceleration - local_min_acceleration) / number_of_acceleration_candidates;
        step_acceleration > local_epsilon) {
      for (std::size_t i = 1; i < number_of_acceleration_candidates; ++i) {
        considerCandidate(local_min_acceleration + i * step_acceleration);
      }
    }

    if (best_acceleration.has_value()) {
      if (min_time_diff <= -step_time + step_time_tolerance) {
        return local_max_acceleration;
      } else if (min_time_diff >= step_time - step_time_tolerance) {
        return local_min_acceleration;
      } else if (
        std::abs(min_time_diff) < step_time && min_distance_diff > predicted_distance_tolerance) {
        throw ControllerError(
          "It is not possible to stop with the expected accuracy of the distance"
          " from the waypoint - the trajectory does not meet the constraints. Check if the "
          "specified "
          "time is not too short for the constraints involved.");
      } else {
        return best_acceleration.value();
      }
    } else {
      throw ControllerError(
        "No acceleration found, speed: ", speed, ", acceleration: ", acceleration,
        ", remaining_time: ", remaining_time, ", remaining_time_source ", remaining_time_source,
        ", remaining_distance: ", remaining_distance, ". ", *this);
    }
  }
}
}  // namespace follow_trajectory
}  // namespace traffic_simulator
