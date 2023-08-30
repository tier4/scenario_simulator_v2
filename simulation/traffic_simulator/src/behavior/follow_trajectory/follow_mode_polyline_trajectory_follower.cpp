#include <geometry/vector3/hypot.hpp>
#include <geometry/vector3/operator.hpp>
#include <iostream>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/behavior/follow_trajectory/follow_mode_polyline_trajectory_follower.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{

auto FollowModePolylineTrajectoryFollower::setParameters(
  [[maybe_unused]] const traffic_simulator_msgs::msg::EntityStatus & entity_status,
  [[maybe_unused]] const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter,
  [[maybe_unused]] const double step_time) -> void
{
  throw common::Error(
    "Unable to set parameters for FollowModeTrajectoryFollower without vehicle_parameters, ",
    "entity name: ", std::quoted(entity_status.name));
}

auto FollowModePolylineTrajectoryFollower::setParameters(
  const traffic_simulator_msgs::msg::EntityStatus & entity_status,
  const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter, const double step_time,
  const traffic_simulator_msgs::msg::VehicleParameters & vehicle_parameters) -> void
{
  vehicle = std::make_unique<Vehicle>(entity_status, behavior_parameter, vehicle_parameters);
  previous_target = vehicle->getFrontPosition();
  step_time_m = step_time;
}

std::optional<traffic_simulator_msgs::msg::EntityStatus>
FollowModePolylineTrajectoryFollower::followTrajectory(
  std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & polyline_trajectory)
{
  if (polyline_trajectory->shape.vertices.empty()) {
    return std::nullopt;
  }

  polyline_trajectory_m = polyline_trajectory;

  auto target_and_speed_data = getTargetPositionAndDesiredSpeed(vehicle->getFrontPosition());

  if (!target_and_speed_data) {
    return followTrajectory(polyline_trajectory_m);
  }

  const auto [target_position, desired_speed] = target_and_speed_data.value();

  const auto desired_steering = getSteering(target_position, desired_speed);

  return vehicle->createUpdatedStatus(desired_steering, desired_speed, step_time_m);
}

auto FollowModePolylineTrajectoryFollower::getDistanceAndTimeToFrontWaypoint(
  const geometry_msgs::msg::Point & target_position,
  const geometry_msgs::msg::Point & position) const -> std::optional<std::tuple<double, double>>
{
  // @todo Calculate proper distance and time to target, considering also the curvature of the path.

  const auto [distance_to_front_waypoint, remaining_time_to_front_waypoint] = std::make_tuple(
    math::geometry::hypot(position, target_position),  // TODO DIFFERENT DISTANCE CALCULATION
    (not std::isnan(polyline_trajectory_m->base_time) ? polyline_trajectory_m->base_time : 0.0) +
      polyline_trajectory_m->shape.vertices.front().time - vehicle->getTime());
  /*
       This clause is to avoid division-by-zero errors in later clauses with
       distance_to_front_waypoint as the denominator if the distance
       miraculously becomes zero.
    */
  // @todo Remove the magic number 2.0
  if (std::abs(distance_to_front_waypoint) <= 2.0) {
    return std::nullopt;
  }
  return std::make_tuple(distance_to_front_waypoint, remaining_time_to_front_waypoint);
}

auto FollowModePolylineTrajectoryFollower::getDistanceAndTimeToWaypointWithSpecifiedTime(
  double distance_to_front_waypoint) const -> std::optional<std::tuple<double, double>>
{
  // @todo Same as above - consider the curvature of the path when calculating distance
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
          /*
            @todo This condition allows the vehicle to ignore the time limit.
            It is done in this way because the distance is not calculated properly
            and the velocity is not accurate. If the vehicle reaches the time
            limit step_time_m is added to the time.
        */
          remaining_time < -step_time_m) {
        return std::make_tuple(
          distance_to_front_waypoint +
            total_distance_to(first_waypoint_with_arrival_time_specified),
          remaining_time + step_time_m);
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
  // @todo Remove the magic number 2.0
  if (std::abs(distance) <= 2.0) {
    return std::nullopt;
  }
  return std::make_tuple(distance, remaining_time);
}

void FollowModePolylineTrajectoryFollower::discardTheFrontWaypointFromTrajectory()
{
  //        The OpenSCENARIO standard does not define the behavior when the value of
  //        Timing.domainAbsoluteRelative is "relative". The standard only states
  //        "Definition of time value context as either absolute or relative", and
  //        it is completely unclear when the relative time starts.

  //        This implementation has interpreted the specification as follows:
  //        Relative time starts from the start of FollowTrajectoryAction or from
  //        the time of reaching the previous "waypoint with arrival time".
  //

  if (
    not std::isnan(polyline_trajectory_m->base_time) and
    not std::isnan(polyline_trajectory_m->shape.vertices.front().time)) {
    polyline_trajectory_m->base_time = vehicle->getTime();
  }

  previous_target = polyline_trajectory_m->shape.vertices.front().position.position;

  if (std::rotate(
        std::begin(polyline_trajectory_m->shape.vertices),
        std::begin(polyline_trajectory_m->shape.vertices) + 1,
        std::end(polyline_trajectory_m->shape.vertices));
      not polyline_trajectory_m->closed) {
    polyline_trajectory_m->shape.vertices.pop_back();
  }
}

auto FollowModePolylineTrajectoryFollower::getSteering(
  const geometry_msgs::msg::Point & target_position, double desired_speed) const -> double
{
  using math::geometry::operator-;
  using math::geometry::operator*;
  using math::geometry::operator+;

  auto a = previous_target.value().y - target_position.y;
  auto b = target_position.x - previous_target.value().x;
  auto c =
    previous_target.value().x * target_position.y - target_position.x * previous_target.value().y;

  auto gain = 1;
  auto parameter = 1;
  auto heading_error =
    std::remainder(std::atan2(-1 * a, b) - vehicle->getOrientation().z, 2 * M_PI);
  auto cross_track_error =
    -1 * (a * vehicle->getFrontPosition().x + b * vehicle->getFrontPosition().y + c) /
    std::sqrt(a * a + b * b);

  auto cross_track_steering = std::atan2(gain * cross_track_error, (parameter + desired_speed));
  auto steering_input = heading_error + cross_track_steering;

  steering_input =
    std::clamp(steering_input, vehicle->getMaxSteering() * -1, vehicle->getMaxSteering());

  if (any(is_infinity_or_nan, steering_input)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(vehicle->getName()), "'s steering input contains NaN or infinity. The value is ",
      steering_input, ".");
  }

  return steering_input;
}

}  // namespace follow_trajectory
}  // namespace traffic_simulator
