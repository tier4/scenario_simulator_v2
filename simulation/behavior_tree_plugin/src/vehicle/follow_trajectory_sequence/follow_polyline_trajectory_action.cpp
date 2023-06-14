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

#include <behavior_tree_plugin/vehicle/follow_trajectory_sequence/follow_polyline_trajectory_action.hpp>
#include <scenario_simulator_exception/exception.hpp>

#define LINE() \
  std::cout << "\x1b[33m" __FILE__ "\x1b[31m:\x1b[36m" << __LINE__ << "\x1b[0m" << std::endl

#define PRINT(...) std::cout << #__VA_ARGS__ " = " << std::boolalpha << (__VA_ARGS__) << std::endl

namespace entity_behavior
{
namespace vehicle
{
auto FollowPolylineTrajectoryAction::calculateWaypoints()
  -> const traffic_simulator_msgs::msg::WaypointsArray
{
  auto waypoints = traffic_simulator_msgs::msg::WaypointsArray();
  waypoints.waypoints.push_back(entity_status.pose.position);
  for (const auto & vertex : parameter->shape.vertices) {
    waypoints.waypoints.push_back(vertex.position.position);
  }
  return waypoints;
}

auto FollowPolylineTrajectoryAction::calculateObstacle(
  const traffic_simulator_msgs::msg::WaypointsArray &)
  -> const std::optional<traffic_simulator_msgs::msg::Obstacle>
{
  /*
     Obstacle avoidance is not implemented for this action.

     If you implement obstacle avoidance for this action, implement this
     virtual function to return the location of any obstacles blocking the path
     of this action's actor. However, this virtual function is currently used
     only for the visualization of obstacle information, so the obstacle
     avoidance algorithm does not necessarily need to use this virtual
     function.
  */
  return std::nullopt;
}

auto FollowPolylineTrajectoryAction::providedPorts() -> BT::PortsList
{
  auto ports = VehicleActionNode::providedPorts();
  ports.emplace(BT::InputPort<Parameter>("polyline_trajectory_parameter"));
  ports.emplace(BT::InputPort<decltype(target_speed)>("target_speed"));
  return ports;
}

template <typename T, typename = void>
struct is_vector3 : public std::false_type
{
};

template <typename T>
struct is_vector3<
  T, std::void_t<decltype(std::declval<T>().x, std::declval<T>().y, std::declval<T>().z)>>
: public std::true_type
{
};

template <
  typename T, typename U,
  std::enable_if_t<std::conjunction_v<is_vector3<T>, is_vector3<U>>, std::nullptr_t> = nullptr>
auto operator+(const T & a, const U & b)
{
  geometry_msgs::msg::Vector3 v;
  v.x = a.x + b.x;
  v.y = a.y + b.y;
  v.z = a.z + b.z;
  return v;
}

template <
  typename T, typename U,
  std::enable_if_t<std::conjunction_v<is_vector3<T>, is_vector3<U>>, std::nullptr_t> = nullptr>
auto operator-(const T & a, const U & b)
{
  geometry_msgs::msg::Vector3 v;
  v.x = a.x - b.x;
  v.y = a.y - b.y;
  v.z = a.z - b.z;
  return v;
}

template <
  typename T, typename U,
  std::enable_if_t<std::conjunction_v<is_vector3<T>, is_vector3<U>>, std::nullptr_t> = nullptr>
auto operator+=(T & a, const U & b) -> decltype(auto)
{
  a.x += b.x;
  a.y += b.y;
  a.z += b.z;
  return a;
}

template <
  typename T, typename U,
  std::enable_if_t<std::conjunction_v<is_vector3<T>, std::is_scalar<U>>, std::nullptr_t> = nullptr>
auto operator*(const T & a, const U & b)
{
  geometry_msgs::msg::Vector3 v;
  v.x = a.x * b;
  v.y = a.y * b;
  v.z = a.z * b;
  return v;
}

template <
  typename T, typename U,
  std::enable_if_t<std::conjunction_v<is_vector3<T>, std::is_scalar<U>>, std::nullptr_t> = nullptr>
auto operator/(const T & a, const U & b)
{
  geometry_msgs::msg::Vector3 v;
  v.x = a.x / b;
  v.y = a.y / b;
  v.z = a.z / b;
  return v;
}

template <typename T, typename U>
auto hypot(const T & from, const U & to)
{
  return std::hypot(to.x - from.x, to.y - from.y, to.z - from.z);
}

auto norm(const geometry_msgs::msg::Vector3 & v) { return std::hypot(v.x, v.y, v.z); }

auto normalize(const geometry_msgs::msg::Vector3 & v) { return v / norm(v); }

template <typename T, typename U>
auto truncate(const T & v, const U & max)
{
  if (auto x = norm(v); max < x) {
    return v * (max / x);
  } else {
    return v;
  }
}

template <typename T, typename... Ts>
auto isDefinitelyLessThan(T a, T b, Ts... xs)
{
  auto compare = [](T a, T b) {
    return (b - a) > (std::numeric_limits<T>::epsilon() * std::max(std::abs(a), std::abs(b)));
  };

  if constexpr (0 < sizeof...(Ts)) {
    return compare(a, b) and compare(b, xs...);
  } else {
    return compare(a, b);
  }
}

template <typename T>
auto isApproximatelyEqualTo(T a, T b)
{
  return std::abs(a - b) <=
         (std::numeric_limits<T>::epsilon() * std::max(std::abs(a), std::abs(b)));
}

template <typename T>
auto isDefinitelyGreaterThan(T a, T b)
{
  return (a - b) > (std::numeric_limits<T>::epsilon() * std::max(std::abs(a), std::abs(b)));
}

template <typename T>
auto isEssentiallyEqualTo(T a, T b)
{
  return std::abs(a - b) <=
         (std::numeric_limits<T>::epsilon() * std::min(std::abs(a), std::abs(b)));
}

auto FollowPolylineTrajectoryAction::tick() -> BT::NodeStatus
{
  /*
     The following code implements the steering behavior known as "seek". See
     "Steering Behaviors For Autonomous Characters" by Craig Reynolds for more
     information.

     See https://www.researchgate.net/publication/2495826_Steering_Behaviors_For_Autonomous_Characters
  */
  if (getBlackBoardValues();
      request != traffic_simulator::behavior::Request::FOLLOW_POLYLINE_TRAJECTORY or
      not getInput<decltype(parameter)>("polyline_trajectory_parameter", parameter) or
      not getInput<decltype(target_speed)>("target_speed", target_speed) or not parameter) {
    return BT::NodeStatus::FAILURE;
  } else if (parameter->shape.vertices.empty()) {
    LINE();
    return BT::NodeStatus::SUCCESS;
  } else {
    const auto position = entity_status.pose.position;

    /*
       We've made sure that parameter->shape.vertices is not empty, so a
       reference to vertices.front() always succeeds. vertices.front() is
       referenced only this once in this member function.
    */
    const auto target_position = parameter->shape.vertices.front().position.position;

    /*
       Note: If not dynamic_constraints_ignorable, the linear distance should
       cause problems.
    */
    const auto distance_to_front_waypoint = hypot(position, target_position);  // [m]

    const auto remaining_time_to_front_waypoint =
      (parameter->base_time ? *parameter->base_time : 0.0) +
      parameter->shape.vertices.front().time - entity_status.time;

    const auto [distance, remaining_time] = [&]() {
      if (const auto first_waypoint_with_arrival_time_specified = std::find_if(
            std::begin(parameter->shape.vertices), std::end(parameter->shape.vertices),
            [](auto && vertex) { return not std::isnan(vertex.time); });
          first_waypoint_with_arrival_time_specified != std::end(parameter->shape.vertices)) {
        /*
           Note for anyone working on adding support for followingMode follow
           to this function (FollowPolylineTrajectoryAction::tick) in the
           future: if followingMode is follow, this distance calculation may be
           inappropriate.
        */
        auto total_distance_to = [this](auto last) {
          auto total_distance = 0.0;
          for (auto iter = std::begin(parameter->shape.vertices); 0 < std::distance(iter, last);
               ++iter) {
            total_distance += hypot(iter->position.position, std::next(iter)->position.position);
          }
          return total_distance;
        };

        if (const auto remaining_time = (parameter->base_time ? *parameter->base_time : 0.0) +
                                        first_waypoint_with_arrival_time_specified->time -
                                        entity_status.time;
            /*
               The conditional expression below should ideally be
               remaining_time < 0.

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
            remaining_time < -step_time) {
          throw common::Error(
            "Vehicle ", std::quoted(entity_status.name),
            " failed to reach the trajectory waypoint at the specified time. The specified time "
            "is ",
            first_waypoint_with_arrival_time_specified->time, " seconds (",
            (parameter->base_time ? "absolute" : "relative"),
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

    const auto acceleration = entity_status.action_status.accel.linear.x;  // [m/s^2]

    const auto max_acceleration = std::min(
      acceleration /* [m/s^2] */ +
        behavior_parameter.dynamic_constraints.max_acceleration_rate /* [m/s^3] */ *
          step_time /* [s] */,
      +behavior_parameter.dynamic_constraints.max_acceleration /* [m/s^2] */);

    const auto min_acceleration = std::max(
      acceleration /* [m/s^2] */ -
        behavior_parameter.dynamic_constraints.max_deceleration_rate /* [m/s^3] */ *
          step_time /* [s] */,
      -behavior_parameter.dynamic_constraints.max_deceleration /* [m/s^2] */);

    const auto speed = entity_status.action_status.twist.linear.x;  // [m/s]

    /*
       The desired acceleration is the acceleration at which the destination
       can be reached exactly at the specified time (= time remaining at zero).

       If no arrival time is specified for subsequent waypoints, there is no
       need to accelerate or decelerate, so the current acceleration will be
       the desired speed.
    */
    const auto desired_acceleration = [&]() {
      if (std::isinf(remaining_time)) {
        return acceleration;
      } else {
        /*
                        v [m/s]
                         ^
                         |
           desired_speed +   /|
                         |  / |
                         | /  |
                   speed +/   |
                         |    |
                         |    |
                         +----+-------------> t [s]
                       0     remaining_time

           desired_speed = speed + desired_acceleration * remaining_time

           distance = (speed + desired_speed) * remaining_time * 1/2

                    = (speed + speed + desired_acceleration * remaining_time) * remaining_time * 1/2

                    = speed * remaining_time + desired_acceleration * remaining_time^2 * 1/2
        */
        return 2 * distance / std::pow(remaining_time, 2) - 2 * speed / remaining_time;
      }
    }();

    /*
       However, the desired acceleration is unrealistically large in terms of
       vehicle performance and dynamic constraints, so it is clamped to a
       realistic value.
    */
    const auto desired_speed =
      speed + std::clamp(desired_acceleration, min_acceleration, max_acceleration) * step_time;

    const auto remaining_time_to_arrival_to_front_waypoint =
      distance_to_front_waypoint / desired_speed;  // [s]

    if constexpr (true) {
      // clang-format off
      std::cout << std::string(80, '-') << std::endl;

      std::cout << std::fixed << "acceleration = " << acceleration << std::endl;

      std::cout << std::fixed
                << "min_acceleration "
                << "== std::max(acceleration - max_deceleration_rate * step_time, -max_deceleration) "
                << "== std::max(" << acceleration << " - " << behavior_parameter.dynamic_constraints.max_deceleration_rate << " * " << step_time << ", " << -behavior_parameter.dynamic_constraints.max_deceleration << ") "
                << "== std::max(" << acceleration << " - " << behavior_parameter.dynamic_constraints.max_deceleration_rate * step_time << ", " << -behavior_parameter.dynamic_constraints.max_deceleration << ") "
                << "== std::max(" << (acceleration - behavior_parameter.dynamic_constraints.max_deceleration_rate * step_time) << ", " << -behavior_parameter.dynamic_constraints.max_deceleration << ") "
                << "== " << min_acceleration
                << std::endl;

      std::cout << std::fixed
                << "max_acceleration "
                << "== std::min(acceleration + max_acceleration_rate * step_time, +max_acceleration) "
                << "== std::min(" << acceleration << " + " << behavior_parameter.dynamic_constraints.max_acceleration_rate << " * " << step_time << ", " << behavior_parameter.dynamic_constraints.max_acceleration << ") "
                << "== std::min(" << acceleration << " + " << behavior_parameter.dynamic_constraints.max_acceleration_rate * step_time << ", " << behavior_parameter.dynamic_constraints.max_acceleration << ") "
                << "== std::min(" << (acceleration + behavior_parameter.dynamic_constraints.max_acceleration_rate * step_time) << ", " << behavior_parameter.dynamic_constraints.max_acceleration << ") "
                << "== " << max_acceleration
                << std::endl;

      std::cout << std::fixed
                << "min_acceleration < acceleration < max_acceleration "
                << "== " << min_acceleration << " < " << acceleration << " < " << max_acceleration << std::endl;

      std::cout << std::fixed << std::boolalpha
                << "desired_acceleration "
                << "== 2 * distance / std::pow(remaining_time, 2) - 2 * speed / remaining_time "
                << "== 2 * " << distance << " / " << std::pow(remaining_time, 2) << " - 2 * " << speed << " / " << remaining_time << " "
                << "== " << (2 * distance / std::pow(remaining_time, 2)) << " - " << (2 * speed / remaining_time) << " "
                << "== " << desired_acceleration << " "
                << "(acceleration < desired_acceleration == " << (acceleration < desired_acceleration) << " == need to " <<(acceleration < desired_acceleration ? "accelerate" : "decelerate") << ")"
                << std::endl;

      std::cout << std::fixed
                << "desired_speed "
                << "== speed + std::clamp(desired_acceleration, min_acceleration, max_acceleration) * step_time "
                << "== " << speed << " + std::clamp(" << desired_acceleration << ", " << min_acceleration << ", " << max_acceleration << ") * " << step_time << " "
                << "== " << speed << " + " << std::clamp(desired_acceleration, min_acceleration, max_acceleration) << " * " << step_time << " "
                << "== " << speed << " + " << std::clamp(desired_acceleration, min_acceleration, max_acceleration) * step_time << " "
                << "== " << desired_speed
                << std::endl;

      PRINT(distance_to_front_waypoint);
      PRINT(remaining_time_to_front_waypoint);

      PRINT(distance);
      PRINT(remaining_time);

      std::cout << std::fixed
                << "remaining_time_to_arrival_to_front_waypoint "
                << "("
                << "== distance_to_front_waypoint / desired_speed "
                << "== " << distance_to_front_waypoint << " / " << desired_speed << " "
                << "== " << remaining_time_to_arrival_to_front_waypoint
                << ")"
                << std::endl;

      std::cout << std::fixed
                << "arrive during this frame? "
                << "== remaining_time_to_arrival_to_front_waypoint < step_time "
                << "== " << remaining_time_to_arrival_to_front_waypoint << " < " << step_time << " "
                << "== " << std::boolalpha << isDefinitelyLessThan(remaining_time_to_arrival_to_front_waypoint, step_time)
                << std::endl;

      std::cout << std::fixed << std::boolalpha
                << "not too early? "
                << "== std::isnan(remaining_time_to_front_waypoint) or remaining_time_to_front_waypoint < remaining_time_to_arrival_to_front_waypoint + step_time "
                << "== std::isnan(" << remaining_time_to_front_waypoint << ") or " << remaining_time_to_front_waypoint << " < " << remaining_time_to_arrival_to_front_waypoint << " + " << step_time << " "
                << "== " << std::isnan(remaining_time_to_front_waypoint) << " or " << isDefinitelyLessThan(remaining_time_to_front_waypoint, remaining_time_to_arrival_to_front_waypoint + step_time) << " "
                << "== " << (std::isnan(remaining_time_to_front_waypoint) or isDefinitelyLessThan(remaining_time_to_front_waypoint, remaining_time_to_arrival_to_front_waypoint + step_time))
                << std::endl;
      // clang-format on
    }

    /*
       If the target point is reached during this step, it is considered
       reached.
    */
    if (
      std::isnan(remaining_time_to_arrival_to_front_waypoint) or
      isDefinitelyLessThan(remaining_time_to_arrival_to_front_waypoint, step_time)) {
      /*
         The condition "Is remaining time to front waypoint less than remaining
         time to arrival to front waypoint + step time?" means "If arrival is
         next frame, is it too late?". This clause is executed only if the
         front waypoint is expected to arrive during this frame. That is, the
         conjunction of these conditions means "Did the vehicle arrive at the
         front waypoint exactly on time?" Otherwise the vehicle will have
         reached the front waypoint too early.

         This means that the vehicle did not slow down enough to reach the
         current waypoint after passing the previous waypoint. In order to cope
         with such situations, it is necessary to perform speed planning
         considering not only the front of the waypoint queue but also the
         waypoints after it. However, even with such speed planning, there is a
         possibility that on-time arrival may not be possible depending on the
         relationship between waypoint intervals, specified arrival times,
         vehicle parameters, and dynamic restraints. For example, there is a
         situation in which a large speed change is required over a short
         distance while the permissible jerk is small.

         This implementation does simple velocity planning that considers only
         the nearest waypoints in favor of simplicity of implementation.

         Note: There is no need to consider the case of arrival too late.
         Because that case has already been verified when calculating the
         remaining time.

         Note: If remaining time to front waypoint is nan, there is no need to
         verify whether the arrival is too early. This arrival determination is
         only interesting for the front waypoint. Verifying whether or not the
         arrival time is specified in the front waypoint is exactly the
         condition of "Is remaining time to front waypoint nan?"
      */
      if (
        std::isnan(remaining_time_to_front_waypoint) or
        isDefinitelyLessThan(
          remaining_time_to_front_waypoint,
          remaining_time_to_arrival_to_front_waypoint + step_time)) {
        if (std::rotate(
              std::begin(parameter->shape.vertices), std::begin(parameter->shape.vertices) + 1,
              std::end(parameter->shape.vertices));
            not parameter->closed) {
          parameter->shape.vertices.pop_back();
        }
        /*
           The OpenSCENARIO standard does not define the behavior when the
           value of Timing.domainAbsoluteRelative is "relative". The standard
           only states "Definition of time value context as either absolute or
           relative", and it is completely unclear when the relative time
           starts.

           This implementation has interpreted the specification as follows:
           Relative time starts from the start of FollowTrajectoryAction or
           from the time of reaching the previous "waypoint with arrival time".
        */
        if (parameter->base_time and not std::isnan(remaining_time_to_front_waypoint)) {
          parameter->base_time = entity_status.time;
        }
        return tick();  // tail recursion
      } else {
        throw common::SimulationError(
          "Vehicle ", std::quoted(entity_status.name), " arrived at the waypoint in trajectory ",
          remaining_time_to_front_waypoint,
          " seconds earlier than the specified time. This may be due to unrealistic conditions of "
          "arrival time specification compared to vehicle parameters and dynamic constraints.");
      }
    }

    const auto desired_velocity = [&]() {
      /*
         Note: The followingMode in OpenSCENARIO is passed as variable
         dynamic_constraints_ignorable. the value of the variable is
         `followingMode == position`.
      */
      if (parameter->dynamic_constraints_ignorable) {
        return normalize(target_position - position) * desired_speed;  // [m/s]
      } else {
        /*
           Note: The vector returned if dynamic_constraints_ignorable == true
           ignores parameters such as the maximum rudder angle of the vehicle
           entry. In this clause, such parameters must be respected and the
           rotation angle difference of the z-axis center of the vector must be
           kept below a certain value.
        */
        throw common::SimulationError("The followingMode is only supported for position.");
      }
    }();

    /*
       Note: If obstacle avoidance is to be implemented, the steering behavior
       known by the name "collision avoidance" should be synthesized here into
       steering.
    */
    const auto steering = desired_velocity - velocity;

    velocity = truncate(velocity + steering, desired_speed);

    auto updated_status = entity_status;

    updated_status.pose.position += velocity * step_time;

    updated_status.pose.orientation = [this]() {
      geometry_msgs::msg::Vector3 direction;
      direction.x = 0;
      direction.y = 0;
      direction.z = std::atan2(velocity.y, velocity.x);
      return quaternion_operation::convertEulerAngleToQuaternion(direction);
    }();

    updated_status.action_status.twist.linear.x = norm(velocity);

    updated_status.action_status.twist.linear.y = 0;

    updated_status.action_status.twist.linear.z = 0;

    updated_status.action_status.twist.angular =
      quaternion_operation::convertQuaternionToEulerAngle(quaternion_operation::getRotation(
        entity_status.pose.orientation, updated_status.pose.orientation)) /
      step_time;

    updated_status.action_status.accel.linear =
      (updated_status.action_status.twist.linear - entity_status.action_status.twist.linear) /
      step_time;

    updated_status.action_status.accel.angular =
      (updated_status.action_status.twist.angular - entity_status.action_status.twist.angular) /
      step_time;

    updated_status.time = entity_status.time + step_time;

    updated_status.lanelet_pose_valid = false;

    setOutput("updated_status", updated_status);
    setOutput("waypoints", calculateWaypoints());
    setOutput("obstacle", calculateObstacle(calculateWaypoints()));

    return BT::NodeStatus::RUNNING;
  }
}
}  // namespace vehicle
}  // namespace entity_behavior
