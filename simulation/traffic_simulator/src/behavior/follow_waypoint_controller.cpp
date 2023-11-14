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
  const double remaining_time, const double left_distance, const double acceleration,
  const double speed) const -> double
{
  if (remaining_time <= step_time * 2.0) {
    // speed should already be equal to 0.0 if it is the penultimate step with braking
    if (!with_breaking || (std::abs(speed) < local_epsilon)) {
      return clampAcceleration(0.0, acceleration, speed);
    } else {
      throw ControllerError(
        "The speed in the penultimate step for last waypoint is different from zero, speed: ",
        speed, ", acceleration: ", acceleration, ", remaining_time: ", remaining_time,
        ", left_distance: ", left_distance, ". FollowWaypointController: ", *this);
    }
  } else if (remaining_time <= step_time * 3.0) {
    if (with_breaking) {
      return clampAcceleration(-speed / step_time, acceleration, speed);
    } else {
      double numerator = 2.0 / 3.0 * (left_distance - 2.0 * speed * step_time);
      return clampAcceleration(numerator / std::pow(step_time, 2), acceleration, speed);
    }
  } else if (remaining_time <= step_time * 4.0) {
    if (with_breaking) {
      double numerator = 2.0 * (left_distance - speed * step_time);
      return clampAcceleration(numerator / std::pow(step_time, 2), acceleration, speed);
    }
  }
  throw common::SemanticError(
    "An analytical calculation of acceleration is not available for a time step greater than 4 (",
    step_time * 4, "s) in the case of braking and greater than 3 (", step_time * 3,
    "s) without braking.");
}

auto FollowWaypointController::roundTimeToFullStepsWithTolerance(
  const double remaining_time_source, double time_tolerance) const -> double
{
  const long long int number_of_steps_source =
    static_cast<long long int>(remaining_time_source / step_time);
  const double remaining_time = number_of_steps_source * step_time;
  const double time_diff = (remaining_time_source / step_time) - (remaining_time / step_time);
  if (time_diff > 1.0 - time_tolerance) {
    return remaining_time + step_time + time_tolerance;
  } else {
    return remaining_time + time_tolerance;
  }
}

auto FollowWaypointController::timeRequiredForNonAcceleration(const double acceleration) const
  -> double
{
  const double acc_rate = (acceleration > 0) ? max_deceleration_rate : max_acceleration_rate;
  return (std::abs(acceleration) / (acc_rate * step_time)) * step_time;
}

auto FollowWaypointController::getAccelerationLimits(
  const double acceleration, const double speed) const -> std::pair<double, double>
{
  auto time_for_non_acc = timeRequiredForNonAcceleration(acceleration);
  if (time_for_non_acc < step_time) {
    time_for_non_acc = step_time;
  }

  auto local_min_acc = -max_deceleration;
  if (std::abs(speed) < local_epsilon) {
    local_min_acc = 0.0;
  } else if (time_for_non_acc > 0) {
    local_min_acc = (0.0 - speed) / time_for_non_acc;
  }
  local_min_acc = std::max(local_min_acc, acceleration - max_deceleration_rate * step_time);

  auto local_max_acc = max_acceleration;
  if (std::abs(speed - max_speed) < local_epsilon) {
    local_max_acc = 0.0;
  } else if (time_for_non_acc > 0) {
    local_max_acc = (max_speed - speed) / time_for_non_acc;
  }
  local_max_acc = std::min(local_max_acc, acceleration + max_acceleration_rate * step_time);

  /// @todo
  // such a case occurs, even without braking, it requires further investigation, but this solves it
  if (local_max_acc < local_min_acc) {
    return {local_max_acc, local_max_acc};
  }

  // check the validity of the limits
  double v_min = speed + local_min_acc * step_time;
  double v_max = speed + local_max_acc * step_time;
  if (v_max < -local_epsilon || v_max > max_speed || v_min < -local_epsilon || v_min > max_speed) {
    throw ControllerError(
      "Incorrect acc limits [", local_min_acc, ", ", local_max_acc,
      "] for acceleration: ", acceleration, " and speed: ", speed,
      ". FollowWaypointController: ", *this);
  }
  return {std::max(local_min_acc, -max_deceleration), std::min(local_max_acc, max_acceleration)};
}

auto FollowWaypointController::clampAcceleration(
  const double candidate_acc, const double acceleration, const double speed) const -> double
{
  auto [local_min_acc, local_max_acc] = getAccelerationLimits(acceleration, speed);
  return std::clamp(candidate_acc, local_min_acc, local_max_acc);
}

auto FollowWaypointController::straightMove(
  PredictedState & state, const double candidate_acc) const -> void
{
  state.straightMove(clampAcceleration(candidate_acc, state.acceleration, state.speed), step_time);
}

auto FollowWaypointController::getPredictedWaypointArrivalState(
  const double step_acc, const double remaining_time, const double left_distance,
  const double acceleration, const double speed) const -> std::optional<PredictedState>
{
  // init and first step with acceleration equal to step_acc
  PredictedState state{acceleration, speed, 0.0, 0.0};
  straightMove(state, step_acc);

  if (with_breaking) {
    // calculate the current braking time required for stopping
    PredictedState breaking_check = state;
    while (breaking_check.speed > local_epsilon ||
           std::abs(breaking_check.acceleration) > local_epsilon) {
      if (breaking_check.travel_time >= remaining_time) {
        return std::nullopt;
      }
      straightMove(breaking_check, breaking_check.acceleration - max_deceleration_rate * step_time);
    }
    // if it is breaking time
    if (std::abs(breaking_check.travel_time - remaining_time) <= step_time) {
      return breaking_check;
    }
  }

  // if it is not braking time, more time left for driving with constant speed
  while (std::abs(state.acceleration) > 0) {
    if (state.travel_time >= remaining_time) {
      throw ControllerError(
        "It is not the braking time, but there is no time to achieve acceleration equal "
        "to 0.0 - the trajectory does not meet the constraint of having an acceleration "
        "equal to 0 on arrival at the followed waypoint, speed: ",
        speed, ", acceleration: ", acceleration, ", remaining_time: ", remaining_time,
        ", left_distance: ", left_distance, " step_acc: ", step_acc,
        ". FollowWaypointController: ", *this);
    }
    straightMove(state, 0.0);
  }
  double const_speed_value = state.speed;

  if (with_breaking) {
    // calculate the braking time required for stopping from the current constant speed
    while ((state.speed > local_epsilon || std::abs(state.acceleration) > local_epsilon)) {
      if (state.travel_time >= remaining_time) {
        return std::nullopt;
      }
      straightMove(state, state.acceleration - max_deceleration_rate * step_time);
    }
    // if it is breaking time
    if (std::abs(state.travel_time - remaining_time) <= step_time) {
      return state;
    }
  }

  // count the distance and time of movement with constant speed, use this to prediction
  long double const_speed_distance = left_distance - state.traveled_distance;
  long double const_speed_time = const_speed_distance / const_speed_value;

  if (
    std::abs(const_speed_value) <= std::numeric_limits<double>::epsilon() ||
    const_speed_time < 0.0) {
    return PredictedState{
      state.acceleration, state.speed, std::numeric_limits<double>::max(),
      std::numeric_limits<double>::max()};
  }

  // calc using multiple of the step_time value
  const_speed_time = roundTimeToFullStepsWithTolerance(const_speed_time, step_time_tolerance);
  const_speed_distance = const_speed_time * const_speed_value;
  return PredictedState{
    state.acceleration, state.speed,
    static_cast<double>(const_speed_distance) + state.traveled_distance,
    static_cast<double>(const_speed_time) + state.travel_time};
}

auto FollowWaypointController::getAcceleration(
  const double remaining_time_source, const double left_distance, const double acceleration,
  const double speed) const -> double
{
  // ensure correct counting of the remaining number of steps
  // without this, inaccuracy sometimes results in a decrease of more than 1 step
  // which can cause major difficulties if we want to arrive at the waypoint at the precise time
  const double remaining_time =
    roundTimeToFullStepsWithTolerance(remaining_time_source, step_time_tolerance);

  // if last steps (4 with breaking, 3 without)
  if (with_breaking && remaining_time <= step_time * 4) {
    return getAnalyticalAccelerationForLastSteps(
      remaining_time, left_distance, acceleration, speed);
  } else if (remaining_time <= step_time * 3) {
    return getAnalyticalAccelerationForLastSteps(
      remaining_time, left_distance, acceleration, speed);
  }

  double min_time_diff = std::numeric_limits<double>::max();
  double min_distance_diff = std::numeric_limits<double>::max();
  auto isBetterCandidate = [local_epsilon = this->local_epsilon, &min_time_diff,
                            &min_distance_diff](double time_diff, double distance_diff) -> bool {
    bool same_time = std::abs(time_diff - min_time_diff) < local_epsilon;
    bool time_better = !same_time && (std::abs(time_diff) < std::abs(min_time_diff));
    bool distance_better =
      distance_diff >= 0 &&
      (std::abs(distance_diff) < std::abs(min_distance_diff) || min_distance_diff < 0);
    return time_better || (same_time && distance_better);
  };

  std::optional<double> best_acceleration = std::nullopt;
  auto considerCandidate = [&](double candidate_acc) -> void {
    if (
      auto predicted_state_opt = getPredictedWaypointArrivalState(
        candidate_acc, remaining_time, left_distance, acceleration, speed)) {
      auto time_diff = remaining_time - predicted_state_opt->travel_time;
      auto distance_diff = left_distance - predicted_state_opt->traveled_distance;
      if (isBetterCandidate(time_diff, distance_diff)) {
        min_time_diff = time_diff;
        min_distance_diff = distance_diff;
        best_acceleration = candidate_acc;
      }
    }
  };

  auto [local_min_acc, local_max_acc] = getAccelerationLimits(acceleration, speed);

  // if the range is so tight that there is no choice
  if (std::abs(local_min_acc - local_max_acc) < local_epsilon) {
    return local_min_acc;
  }

  // consider the borderline values and precise value of 0
  considerCandidate(local_min_acc);
  if (local_min_acc < 0.0 && local_max_acc > 0.0) {
    considerCandidate(0.0);
  }
  considerCandidate(local_max_acc);

  // consider intermediate values
  /// @todo
  // it is likely that this search (in the intermediate values) can be skipped
  // if condition (min_time_diff < -step_time || min_time_diff > step_time) is met
  // however, at this time, different calculation results and unexpected changes in acceleration
  // were obtained - this requires further investigation
  double step_acc = (local_max_acc - local_min_acc) / number_of_acceleration_candidates;
  if (step_acc > local_epsilon) {
    for (int i = 1; i < number_of_acceleration_candidates; ++i) {
      considerCandidate(local_min_acc + i * step_acc);
    }
  }

  if (!best_acceleration.has_value()) {
    throw ControllerError(
      "No acceleration found, speed: ", speed, ", acceleration: ", acceleration,
      ", remaining_time: ", remaining_time, ", remaining_time_source ", remaining_time_source,
      ", left_distance: ", left_distance, ". FollowWaypointController: ", *this);
  }

  if (min_time_diff <= -step_time + step_time_tolerance) {
    return local_max_acc;
  } else if (min_time_diff >= step_time - step_time_tolerance) {
    return local_min_acc;
  } else if (
    std::abs(min_time_diff) < step_time && min_distance_diff > predicted_distance_tolerance) {
    throw ControllerError(
      "It is not possible to stop with the expected accuracy of the distance"
      " from the waypoint - the trajectory does not meet the constraints.");
  } else {
    return best_acceleration.value();
  }
}
}  // namespace follow_trajectory
}  // namespace traffic_simulator
