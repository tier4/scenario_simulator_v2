#include <arithmetic/floating_point/comparison.hpp>
#include <geometry/vector3/hypot.hpp>
#include <geometry/vector3/norm.hpp>
#include <geometry/vector3/normalize.hpp>
#include <geometry/vector3/operator.hpp>
#include <geometry/vector3/truncate.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/behavior/follow_trajectory/position_mode_polyline_trajectory_follower.hpp>

#include <iostream>

namespace traffic_simulator
{
namespace follow_trajectory
{
auto PositionModePolylineTrajectoryFollower::setParameters(
  const traffic_simulator_msgs::msg::EntityStatus & entity_status,
  const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter, const double step_time)
  -> void
{
  vehicle = std::make_unique<Vehicle>(entity_status);
  behavior_parameter_m = behavior_parameter;
  step_time_m = step_time;
}

auto PositionModePolylineTrajectoryFollower::setParameters(
  const traffic_simulator_msgs::msg::EntityStatus & entity_status,
  const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter, const double step_time,
  [[maybe_unused]] const traffic_simulator_msgs::msg::VehicleParameters & vehicle_parameters)
  -> void
{
  setParameters(entity_status, behavior_parameter, step_time);
}

std::optional<traffic_simulator_msgs::msg::EntityStatus>
PositionModePolylineTrajectoryFollower::followTrajectory(
  std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & polyline_trajectory)
{
  if (polyline_trajectory->shape.vertices.empty()) {
    return std::nullopt;
  }

  polyline_trajectory_m = polyline_trajectory;

  auto target_and_speed_data = getTargetPositionAndDesiredSpeed();

  if (!target_and_speed_data) {
    return followTrajectory(polyline_trajectory_m);
  }

  const auto [target_position, desired_speed] = target_and_speed_data.value();

  const auto desired_velocity =
    getDesiredVelocity(target_position, vehicle->getCurrentPosition(), desired_speed);

  const auto velocity = getUpdatedVelocity(desired_velocity, desired_speed);

  return vehicle->createUpdatedStatus(velocity, step_time_m);
}

auto PositionModePolylineTrajectoryFollower::getUpdatedVelocity(
  const geometry_msgs::msg::Vector3 & desired_velocity, double desired_speed) const
  -> geometry_msgs::msg::Vector3
{
  using math::geometry::operator*;
  using math::geometry::operator-;
  using math::geometry::operator+;

  const auto current_velocity = vehicle->getOrientation() * vehicle->getCurrentSpeed();

  /*
       Note: If obstacle avoidance is to be implemented, the steering behavior
       known by the name "collision avoidance" should be synthesized here into
       steering.
    */
  const auto steering = desired_velocity - current_velocity;

  const auto velocity = math::geometry::truncate(current_velocity + steering, desired_speed);

  return velocity;
}

auto PositionModePolylineTrajectoryFollower::getDesiredVelocity(
  const geometry_msgs::msg::Point & target_position, const geometry_msgs::msg::Point & position,
  double desired_speed) const -> geometry_msgs::msg::Vector3
{
  using math::geometry::operator-;
  using math::geometry::operator*;

  const geometry_msgs::msg::Vector3 desired_velocity =
    math::geometry::normalize(target_position - position) * desired_speed;

  if (any(is_infinity_or_nan, desired_velocity)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(vehicle->getName()),
      "'s desired velocity contains NaN or infinity. The value is [", desired_velocity.x, ", ",
      desired_velocity.y, ", ", desired_velocity.z, "].");
  }

  return desired_velocity;
}

auto PositionModePolylineTrajectoryFollower::getDistanceAndTimeToFrontWaypoint(
  const geometry_msgs::msg::Point & target_position,
  const geometry_msgs::msg::Point & position) const -> std::optional<std::tuple<double, double>>
{
  const auto [distance_to_front_waypoint, remaining_time_to_front_waypoint] = std::make_tuple(
    math::geometry::hypot(position, target_position),
    (not std::isnan(polyline_trajectory_m->base_time) ? polyline_trajectory_m->base_time : 0.0) +
      polyline_trajectory_m->shape.vertices.front().time - vehicle->getTime());
  /*
       This clause is to avoid division-by-zero errors in later clauses with
       distance_to_front_waypoint as the denominator if the distance
       miraculously becomes zero.
    */
  if (math::arithmetic::isApproximatelyEqualTo(distance_to_front_waypoint, 0.0)) {
    return std::nullopt;
  }
  return std::make_tuple(distance_to_front_waypoint, remaining_time_to_front_waypoint);
}

auto PositionModePolylineTrajectoryFollower::getDistanceAndTimeToWaypointWithSpecifiedTime(
  double distance_to_front_waypoint) const -> std::optional<std::tuple<double, double>>
{
  const auto [distance, remaining_time] = [&]() {
    if (const auto first_waypoint_with_arrival_time_specified = std::find_if(
          std::begin(polyline_trajectory_m->shape.vertices),
          std::end(polyline_trajectory_m->shape.vertices),
          [](auto && vertex) { return not std::isnan(vertex.time); });
        first_waypoint_with_arrival_time_specified !=
        std::end(polyline_trajectory_m->shape.vertices)) {
      /*
            Note for anyone working on adding support for followingMode follow
            to this function (FollowPolylineTrajectoryAction::tick) in the
            future: if followingMode is follow, this distance calculation may be
            inappropriate.
        */
      auto total_distance_to = [&](auto last) {
        auto total_distance = 0.0;
        for (auto iter = std::begin(polyline_trajectory_m->shape.vertices);
             0 < std::distance(iter, last); ++iter) {
          total_distance +=
            math::geometry::hypot(iter->position.position, std::next(iter)->position.position);
        }
        return total_distance;
      };

      if (const auto remaining_time =
            (not std::isnan(polyline_trajectory_m->base_time) ? polyline_trajectory_m->base_time
                                                              : 0.0) +
            first_waypoint_with_arrival_time_specified->time - vehicle->getTime();
          /*
                The condition below should ideally be remaining_time < 0.

                The simulator runs at a constant frame rate, so the step time is
                1/FPS. If the simulation time is an accumulation of step times
                expressed as rational numbers, times that are integer multiples
                of the frame rate will always be exact integer seconds.
                Therefore, the timing of remaining_time == 0 always exists, and
                the velocity planning of this member function (tick) aims to
                reach the waypoint exactly at that timing. So the ideal timeout
                condition is remaining_time < 0.

                But actually the step time is expressed as a float and the
                simulation time is its accumulation. As a result, it is not
                guaranteed that there will be times when the simulation time is
                exactly zero. For example, remaining_time == -0.00006 and it was
                judged to be out of time.

                For the above reasons, the condition is remaining_time <
                -step_time. In other words, the conditions are such that a delay
                of 1 step time is allowed.
            */
          remaining_time < -step_time_m) {
        throw common::Error(
          "Vehicle ", std::quoted(vehicle->getName()),
          " failed to reach the trajectory waypoint at the specified time. The specified time "
          "is ",
          first_waypoint_with_arrival_time_specified->time, " (in ",
          (not std::isnan(polyline_trajectory_m->base_time) ? "absolute" : "relative"),
          " simulation time). This may be due to unrealistic conditions of arrival time "
          "specification compared to vehicle parameters and dynamic constraints.");
      } else {
        return std::make_tuple(
          distance_to_front_waypoint +
            total_distance_to(first_waypoint_with_arrival_time_specified),
          remaining_time);
      }
    } else {
      return std::make_tuple(distance_to_front_waypoint, std::numeric_limits<double>::infinity());
    }
  }();
  if (math::arithmetic::isApproximatelyEqualTo(distance, 0.0)) {
    return std::nullopt;
  }

  return std::make_tuple(distance, remaining_time);
}

void PositionModePolylineTrajectoryFollower::discardTheFrontWaypointFromTrajectory()
{
  //        The OpenSCENARIO standard does not define the behavior when the value of
  //        Timing.domainAbsoluteRelative is "relative". The standard only states
  //        "Definition of time value context as either absolute or relative", and
  //        it is completely unclear when the relative time starts.

  //        This implementation has interpreted the specification as follows:
  //        Relative time starts from the start of FollowTrajectoryAction or from
  //        the time of reaching the previous "waypoint with arrival time".

  //         Note: not std::isnan(polyline_trajectory_m->base_time) means
  //         "Timing.domainAbsoluteRelative is relative".

  //         Note: not std::isnan(polyline_trajectory_m->shape.vertices.front().time)
  //         means "The waypoint about to be popped is the waypoint with the
  //         specified arrival time".
  //

  if (
    not std::isnan(polyline_trajectory_m->base_time) and
    not std::isnan(polyline_trajectory_m->shape.vertices.front().time)) {
    polyline_trajectory_m->base_time = vehicle->getTime();
  }

  if (std::rotate(
        std::begin(polyline_trajectory_m->shape.vertices),
        std::begin(polyline_trajectory_m->shape.vertices) + 1,
        std::end(polyline_trajectory_m->shape.vertices));
      not polyline_trajectory_m->closed) {
    polyline_trajectory_m->shape.vertices.pop_back();
  }
}

}  // namespace follow_trajectory
}  // namespace traffic_simulator
