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
  return traffic_simulator_msgs::msg::WaypointsArray();
}

auto FollowPolylineTrajectoryAction::calculateObstacle(
  const traffic_simulator_msgs::msg::WaypointsArray &)
  -> const std::optional<traffic_simulator_msgs::msg::Obstacle>
{
  return std::nullopt;
}

auto FollowPolylineTrajectoryAction::providedPorts() -> BT::PortsList
{
  auto && ports = VehicleActionNode::providedPorts();
  ports.emplace(BT::InputPort<Parameter>("polyline_trajectory_parameter"));
  ports.emplace(BT::InputPort<decltype(target_speed)>("target_speed"));
  return std::forward<decltype(ports)>(ports);
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

#define DEFINE_VECTOR3_VS_VECTOR3_BINARY_OPERATOR(OPERATOR)                                       \
  template <                                                                                      \
    typename T, typename U,                                                                       \
    std::enable_if_t<std::conjunction_v<is_vector3<T>, is_vector3<U>>, std::nullptr_t> = nullptr> \
  auto operator OPERATOR(const T & a, const U & b)                                                \
  {                                                                                               \
    geometry_msgs::msg::Vector3 v;                                                                \
    v.x = a.x OPERATOR b.x;                                                                       \
    v.y = a.y OPERATOR b.y;                                                                       \
    v.z = a.z OPERATOR b.z;                                                                       \
    return v;                                                                                     \
  }                                                                                               \
  static_assert(true)

DEFINE_VECTOR3_VS_VECTOR3_BINARY_OPERATOR(+);
DEFINE_VECTOR3_VS_VECTOR3_BINARY_OPERATOR(-);
DEFINE_VECTOR3_VS_VECTOR3_BINARY_OPERATOR(*);
DEFINE_VECTOR3_VS_VECTOR3_BINARY_OPERATOR(/);

#define DEFINE_VECTOR3_VS_VECTOR3_COMPOUND_ASSIGNMENT_OPERATOR(OPERATOR)                          \
  template <                                                                                      \
    typename T, typename U,                                                                       \
    std::enable_if_t<std::conjunction_v<is_vector3<T>, is_vector3<U>>, std::nullptr_t> = nullptr> \
  auto operator OPERATOR(T & a, const U & b)->decltype(auto)                                      \
  {                                                                                               \
    a.x OPERATOR b.x;                                                                             \
    a.y OPERATOR b.y;                                                                             \
    a.z OPERATOR b.z;                                                                             \
    return a;                                                                                     \
  }                                                                                               \
  static_assert(true)

DEFINE_VECTOR3_VS_VECTOR3_COMPOUND_ASSIGNMENT_OPERATOR(+=);
DEFINE_VECTOR3_VS_VECTOR3_COMPOUND_ASSIGNMENT_OPERATOR(-=);
DEFINE_VECTOR3_VS_VECTOR3_COMPOUND_ASSIGNMENT_OPERATOR(*=);
DEFINE_VECTOR3_VS_VECTOR3_COMPOUND_ASSIGNMENT_OPERATOR(/=);

#define DEFINE_VECTOR3_VS_SCALAR_BINARY_OPERATOR(OPERATOR)                                   \
  template <                                                                                 \
    typename T, typename U,                                                                  \
    std::enable_if_t<std::conjunction_v<is_vector3<T>, std::is_scalar<U>>, std::nullptr_t> = \
      nullptr>                                                                               \
  auto operator OPERATOR(const T & a, const U & b)                                           \
  {                                                                                          \
    geometry_msgs::msg::Vector3 v;                                                           \
    v.x = a.x OPERATOR b;                                                                    \
    v.y = a.y OPERATOR b;                                                                    \
    v.z = a.z OPERATOR b;                                                                    \
    return v;                                                                                \
  }                                                                                          \
  static_assert(true)

DEFINE_VECTOR3_VS_SCALAR_BINARY_OPERATOR(+);
DEFINE_VECTOR3_VS_SCALAR_BINARY_OPERATOR(-);
DEFINE_VECTOR3_VS_SCALAR_BINARY_OPERATOR(*);
DEFINE_VECTOR3_VS_SCALAR_BINARY_OPERATOR(/);

#define DEFINE_VECTOR3_VS_SCALAR_COMPOUND_ASSIGNMENT_OPERATOR(OPERATOR)                      \
  template <                                                                                 \
    typename T, typename U,                                                                  \
    std::enable_if_t<std::conjunction_v<is_vector3<T>, std::is_scalar<U>>, std::nullptr_t> = \
      nullptr>                                                                               \
  auto operator OPERATOR(T & a, const U & b)                                                 \
  {                                                                                          \
    a.x OPERATOR b;                                                                          \
    a.y OPERATOR b;                                                                          \
    a.z OPERATOR b;                                                                          \
    return a;                                                                                \
  }                                                                                          \
  static_assert(true)

DEFINE_VECTOR3_VS_SCALAR_COMPOUND_ASSIGNMENT_OPERATOR(+=);
DEFINE_VECTOR3_VS_SCALAR_COMPOUND_ASSIGNMENT_OPERATOR(-=);
DEFINE_VECTOR3_VS_SCALAR_COMPOUND_ASSIGNMENT_OPERATOR(*=);
DEFINE_VECTOR3_VS_SCALAR_COMPOUND_ASSIGNMENT_OPERATOR(/=);

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

template <typename T>
auto definitelyLessThan(T a, T b)
{
  return (b - a) > (std::numeric_limits<T>::epsilon() * std::max(std::abs(a), std::abs(b)));
}

template <typename T>
auto approximatelyEqualTo(T a, T b)
{
  return std::abs(a - b) <=
         (std::numeric_limits<T>::epsilon() * std::max(std::abs(a), std::abs(b)));
}

template <typename T>
auto definitelyGreaterThan(T a, T b)
{
  return (a - b) > (std::numeric_limits<T>::epsilon() * std::max(std::abs(a), std::abs(b)));
}

template <typename T>
auto essentiallyEqualTo(T a, T b)
{
  return std::abs(a - b) <=
         (std::numeric_limits<T>::epsilon() * std::min(std::abs(a), std::abs(b)));
}

auto FollowPolylineTrajectoryAction::tick() -> BT::NodeStatus
{
  auto pop_front = [this]() {
    if (std::rotate(
          std::begin(parameter->shape.vertices), std::begin(parameter->shape.vertices) + 1,
          std::end(parameter->shape.vertices));
        not parameter->closed) {
      parameter->shape.vertices.pop_back();
    }
  };

  if (getBlackBoardValues();
      request == traffic_simulator::behavior::Request::FOLLOW_POLYLINE_TRAJECTORY and
      getInput<decltype(parameter)>("polyline_trajectory_parameter", parameter) and
      getInput<decltype(target_speed)>("target_speed", target_speed) and parameter and
      not parameter->shape.vertices.empty()) {
    const auto position = entity_status.pose.position;

    /*
       We've made sure that parameter->shape.vertices is not empty, so a
       reference to vertices.front() always succeeds. vertices.front() is
       referenced only this once in this member function.
    */
    const auto target_position = parameter->shape.vertices.front().position.position;

    const auto distance = hypot(position, target_position);  // [m]

    const auto remaining_time = [this]() {
      if (const auto iter = std::find_if(
            std::begin(parameter->shape.vertices), std::end(parameter->shape.vertices),
            [this](auto && vertex) { return vertex.time and entity_status.time < *vertex.time; });
          iter != std::end(parameter->shape.vertices)) {
        return *iter->time - entity_status.time;
      } else {
        return std::numeric_limits<double>::infinity();
      }
    }();

    const auto acceleration = entity_status.action_status.accel.linear.x;  // [m/s^2]

    const auto max_acceleration = std::clamp(
      acceleration /* [m/s^2] */ +
        behavior_parameter.dynamic_constraints.max_acceleration_rate /* [m/s^3] */ *
          step_time /* [s] */,
      -behavior_parameter.dynamic_constraints.max_deceleration /* [m/s^2] */,
      +behavior_parameter.dynamic_constraints.max_acceleration /* [m/s^2] */);

    const auto max_deceleration = std::clamp(
      acceleration /* [m/s^2] */ -
        behavior_parameter.dynamic_constraints.max_deceleration_rate /* [m/s^3] */ *
          step_time /* [s] */,
      -behavior_parameter.dynamic_constraints.max_deceleration /* [m/s^2] */,
      +behavior_parameter.dynamic_constraints.max_acceleration /* [m/s^2] */);

    const auto speed = entity_status.action_status.twist.linear.x;  // [m/s]

    /*
       The desired speed is the speed at which the destination can be reached
       exactly at the specified time (= time remaining at zero).

       If no arrival time is specified for subsequent waypoints, there is no
       need to accelerate or decelerate, so the current speed remains the
       desired speed.
    */
    const auto desired_speed = std::isinf(remaining_time) ? speed : distance / remaining_time;

    /*
       However, the desired speed is unrealistically large in terms of vehicle
       performance and dynamic constraints, so it is clamped to a realistic
       value.
    */
    const auto max_speed = std::clamp(
      desired_speed,  // [m/s]
      speed /* [m/s] */ - max_deceleration /* [m/s^2] */ * step_time,
      speed /* [m/s] */ + max_acceleration /* [m/s^2] */ * step_time);

    const auto remaining_time_to_arrival = distance / max_speed;  // [s]

    std::cout << "remaining_time_to_arrival = " << remaining_time_to_arrival << " / "
              << remaining_time << std::endl;

    /*
       If the target point is reached during this step, it is considered
       reached.
    */
    if (definitelyLessThan(remaining_time_to_arrival, step_time)) {
      /*
         If remaining time to arrival is less than remaining time, the vehicle
         reached the waypoint on time. Otherwise, the vehicle has reached the
         waypoint earlier than the specified time.

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
      */
      std::cout << "REACHED!" << std::endl;
      if (definitelyLessThan(remaining_time_to_arrival, remaining_time)) {
        pop_front();
        return tick();  // tail recursion
      } else {
        throw common::SimulationError(
          "Vehicle ", std::quoted(entity_status.name), " reached the target waypoint ",
          remaining_time,
          " seconds earlier than the specified time. This is not due to a bug in the simulator, "
          "but due to waypoint settings. Since FollowTrajectoryAction respects vehicle parameters "
          "and dynamic constraints to accelerate and decelerate, the distance between waypoints "
          "and the arrival time of waypoints should be specified within a realistic range.");
      }
    }

    const auto desired_velocity = normalize(target_position - position) * max_speed;  // [m/s]

    const auto steering = desired_velocity - velocity;

    velocity = truncate(velocity + steering, max_speed);

    const auto previous_direction = std::exchange(direction, [this]() {
      geometry_msgs::msg::Vector3 direction;
      direction.x = 0;
      direction.y = 0;
      direction.z = std::atan2(velocity.y, velocity.x);
      return direction;
    }());

    auto updated_status = entity_status;

    // clang-format off
    updated_status.action_status.twist.linear.x = norm(velocity);
    updated_status.action_status.twist.linear.y = 0;
    updated_status.action_status.twist.linear.z = 0;
    updated_status.action_status.twist.angular.x = 0;
    updated_status.action_status.twist.angular.y = 0;
    updated_status.action_status.twist.angular.z = (direction.z - previous_direction.z) / step_time;
    updated_status.action_status.accel.linear = (updated_status.action_status.twist.linear - entity_status.action_status.twist.linear) / step_time;
    updated_status.action_status.accel.angular = (updated_status.action_status.twist.angular - entity_status.action_status.twist.angular) / step_time;
    updated_status.pose.position += velocity * step_time;
    updated_status.pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(direction);
    updated_status.time = entity_status.time + step_time;
    updated_status.lanelet_pose_valid = false;
    // clang-format on

    setOutput("updated_status", updated_status);
    setOutput("waypoints", calculateWaypoints());
    setOutput("obstacle", calculateObstacle(calculateWaypoints()));

    return parameter->shape.vertices.empty() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}
}  // namespace vehicle
}  // namespace entity_behavior
