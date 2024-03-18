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

#include <traffic_simulator/behavior/follow_waypoint_controller.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{
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
  } else if (remaining_time_source >= std::numeric_limits<std::size_t>::max() * step_time) {
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
  return (std::abs(acceleration) / (acceleration_rate * step_time)) * step_time;
}

auto FollowWaypointController::getAccelerationLimits(
  const double acceleration, const double speed) const -> std::pair<double, double>
{
  const auto time_for_non_acceleration = [&, acceleration]() {
    if (std::abs(acceleration) < local_epsilon) {
      return 0.0;
    } else {
      auto result = getTimeRequiredForNonAcceleration(acceleration);
      return (result < step_time) ? step_time : result;
    }
  }();

  const auto local_min_acceleration = [&]() {
    const auto local_min_acceleration = acceleration - max_deceleration_rate * step_time;
    if (std::abs(speed) < local_epsilon) {
      // If the speed is equal to 0.0, it should no longer be decreased.
      return std::max(0.0, local_min_acceleration);
    } else if (time_for_non_acceleration > local_epsilon) {
      // If the acceleration is not 0.0, ensure that there will be sufficient time to set it to 0.0.
      return std::max(-speed / time_for_non_acceleration, local_min_acceleration);
    } else {
      /*
         Otherwise, return an acceleration limited by constraints: it cannot be less than
         -max_deceleration, it cannot be less than local_min_acceleration (resulting from the max
         deceleration rate) and it cannot be less than -speed/step_time as this would result in a
         negative speed.
      */
      return std::max(-max_deceleration, std::max(local_min_acceleration, -speed / step_time));
    }
  }();

  const auto local_max_acceleration = [&]() {
    const auto local_max_acceleration = acceleration + max_acceleration_rate * step_time;
    if (std::abs(speed - target_speed) < local_epsilon) {
      // If the speed is equal to target_speed, it should no longer be increased.
      return std::min(0.0, local_max_acceleration);
    } else if (speed > target_speed) {
      // If speed is too high, assume that the max acceleration is equal to the min acceleration.
      return local_min_acceleration;
    } else if (time_for_non_acceleration > local_epsilon) {
      // If the acceleration is not 0.0, ensure that there will be sufficient time to set it to 0.0.
      return std::min((target_speed - speed) / time_for_non_acceleration, local_max_acceleration);
    } else {
      /*
         Otherwise, return an acceleration limited by constraints: it cannot be greater than
         max_acceleration, it cannot be greater than local_max_acceleration (resulting from the max
         acceleration rate) and it cannot be greater than (target_speed-speed)/step_time as this
         would result in a speed greater than target_speed.
      */
      return std::min(
        max_acceleration, std::min(local_max_acceleration, (target_speed - speed) / step_time));
    }
  }();

  /// @todo
  if (local_max_acceleration < local_min_acceleration) {
    // Such a case occurs, even without braking, it requires further investigation - this solves it.
    return {local_max_acceleration, local_max_acceleration};
  } else {
    // Check the validity of the limits.
    const double speed_min = speed + local_min_acceleration * step_time;
    const double speed_max = speed + local_max_acceleration * step_time;
    if (
      speed_max < -local_epsilon || speed_max > std::max(max_speed, target_speed) + local_epsilon ||
      speed_min < -local_epsilon || speed_min > std::max(max_speed, target_speed) + local_epsilon) {
      throw ControllerError(
        "Incorrect acceleration limits [", local_min_acceleration, ", ", local_max_acceleration,
        "] for acceleration: ", acceleration, " and speed: ", speed, " -> speed_min: ", speed_min,
        " speed_max: ", speed_max, ". ", *this);
    } else {
      return {local_min_acceleration, local_max_acceleration};
    }
  }
}

auto FollowWaypointController::clampAcceleration(
  const double candidate_acceleration, const double acceleration, const double speed) const
  -> double
{
  auto [local_min_acceleration, local_max_acceleration] =
    getAccelerationLimits(acceleration, speed);
  return std::clamp(candidate_acceleration, local_min_acceleration, local_max_acceleration);
}

auto FollowWaypointController::moveStraight(
  PredictedState & state, const double candidate_acceleration) const -> void
{
  state.moveStraight(
    clampAcceleration(candidate_acceleration, state.acceleration, state.speed), step_time);
}

auto FollowWaypointController::getPredictedStopStateWithoutConsideringTime(
  const double step_acceleration, const double remaining_distance, const double acceleration,
  const double speed) const -> std::optional<PredictedState>
{
  PredictedState breaking_check{acceleration, speed, 0.0, 0.0};
  moveStraight(breaking_check, step_acceleration);
  while (!breaking_check.isImmobile(local_epsilon)) {
    if (breaking_check.traveled_distance > remaining_distance + predicted_distance_tolerance) {
      return std::nullopt;
    } else {
      moveStraight(breaking_check, breaking_check.acceleration - max_deceleration_rate * step_time);
    }
  }
  return breaking_check;
}

auto FollowWaypointController::getPredictedWaypointArrivalState(
  const double step_acceleration, const double remaining_time, const double remaining_distance,
  const double acceleration, const double speed) const -> std::optional<PredictedState>
{
  const auto brakeUntilImmobility = [&](PredictedState & state) -> bool {
    while (!state.isImmobile(local_epsilon)) {
      if (state.travel_time >= remaining_time) {
        return false;
      } else {
        moveStraight(state, state.acceleration - max_deceleration_rate * step_time);
      }
    }
    return true;
  };

  PredictedState state{acceleration, speed, 0.0, 0.0};

  if (remaining_time < step_time) {
    return state;
  } else {
    // First step with acceleration equal to step_acceleration.
    moveStraight(state, step_acceleration);

    if (with_breaking) {
      // Predict the current (before acceleration zeroing) braking time required for stopping.
      PredictedState breaking_check = state;
      if (!brakeUntilImmobility(breaking_check)) {
        // If complete immobility is not possible - ignore this candidate.
        return std::nullopt;
      } else if (std::abs(breaking_check.travel_time - remaining_time) <= step_time) {
        // If it is breaking time - consider this candidate.
        return breaking_check;
      }
    }

    // If it is not braking time, more time left for driving with constant speed.
    while (std::abs(state.acceleration) > 0.0) {
      if (state.travel_time >= remaining_time) {
        throw ControllerError(
          "It is not the braking time, but there is no time to achieve acceleration equal "
          "to 0.0 - the trajectory does not meet the constraint of having an acceleration "
          "equal to 0.0 on arrival at the followed waypoint, speed: ",
          speed, ", acceleration: ", acceleration, ", remaining_time: ", remaining_time,
          ", remaining_distance: ", remaining_distance, " step_acceleration: ", step_acceleration,
          ". ", *this);
      } else {
        moveStraight(state, 0.0);
      }
    }

    if (std::abs(state.speed) <= local_epsilon) {
      // If the previous steps caused the constant speed to be extremely low - ignore this.
      return std::nullopt;
    } else {
      const double const_speed_value = state.speed;

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
        return PredictedState{
          state.acceleration, state.speed, rounded_const_speed_distance + state.traveled_distance,
          rounded_const_speed_time + state.travel_time};
      }
    }
  }
}

auto FollowWaypointController::getAcceleration(
  const double remaining_distance, const double acceleration, const double speed) const -> double
{
  const auto [local_min_acceleration, local_max_acceleration] =
    getAccelerationLimits(acceleration, speed);

  const double step_acceleration =
    (local_max_acceleration - local_min_acceleration) / number_of_acceleration_candidates;

  auto min_distance_diff = std::numeric_limits<double>::lowest();

  std::optional<double> best_acceleration = std::nullopt;

  for (std::size_t i = 0; i <= number_of_acceleration_candidates; ++i) {
    const double candidate_acceleration = local_min_acceleration + i * step_acceleration;

    if (const auto predicted_state_opt = getPredictedStopStateWithoutConsideringTime(
          candidate_acceleration, remaining_distance, acceleration, speed);
        predicted_state_opt) {
      if (const auto distance_diff = remaining_distance - predicted_state_opt->traveled_distance;
          (distance_diff >= 0 || min_distance_diff < 0) &&
          (std::abs(distance_diff) < std::abs(min_distance_diff))) {
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
  const double remaining_time_source, const double remaining_distance, const double acceleration,
  const double speed) const -> double
{
  const auto [local_min_acceleration, local_max_acceleration] =
    getAccelerationLimits(acceleration, speed);

  if ((speed + local_min_acceleration * step_time) * step_time > remaining_distance) {
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
    return getAcceleration(remaining_distance, acceleration, speed);
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
          candidate_acceleration, remaining_time, remaining_distance, acceleration, speed)) {
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
