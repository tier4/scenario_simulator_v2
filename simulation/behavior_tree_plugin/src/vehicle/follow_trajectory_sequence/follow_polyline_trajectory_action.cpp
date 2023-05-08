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

#define LINE() \
  std::cout << "; \x1b[33m" __FILE__ "\x1b[31m:\x1b[36m" << __LINE__ << "\x1b[0m" << std::endl

#define PRINT(...) \
  std::cout << "; " #__VA_ARGS__ " = " << std::boolalpha << (__VA_ARGS__) << std::endl

namespace entity_behavior
{
namespace vehicle
{
auto FollowPolylineTrajectoryAction::calculateWaypoints()
  -> const traffic_simulator_msgs::msg::WaypointsArray
{
  return traffic_simulator_msgs::msg::WaypointsArray();

  auto && waypoints = traffic_simulator_msgs::msg::WaypointsArray();

  for (auto && vertex : parameter->shape.vertices) {
    auto && point = geometry_msgs::msg::Point();
    point.x = vertex.position.position.x;
    point.y = vertex.position.position.y;
    point.z = vertex.position.position.z;
    waypoints.waypoints.push_back(std::forward<decltype(point)>(point));
  }

  return std::forward<decltype(waypoints)>(waypoints);
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
auto distance(const T & from, const U & to)
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

template <typename T, typename U>
auto truncate(T & v, const U & max)
{
  if (auto x = norm(v); max < x) {
    v *= (max / x);
  }
}

auto FollowPolylineTrajectoryAction::tick() -> BT::NodeStatus
{
  getBlackBoardValues();

  switch (request) {
    case traffic_simulator::behavior::Request::FOLLOW_POLYLINE_TRAJECTORY:
      if (
        getInput<decltype(parameter)>("polyline_trajectory_parameter", parameter) and
        getInput<decltype(target_speed)>("target_speed", target_speed) and
        not parameter->shape.vertices.empty()) {
        // PRINT(parameter->initial_distance_offset);
        // PRINT(parameter->dynamic_constraints_ignorable);
        // PRINT(parameter->closed);

        auto waypoints_exhausted = [&]() { return parameter->shape.vertices.empty(); };

        auto pop_current_waypoint = [&]() {
          if (std::rotate(
                std::begin(parameter->shape.vertices), std::begin(parameter->shape.vertices) + 1,
                std::end(parameter->shape.vertices));
              not parameter->closed) {
            parameter->shape.vertices.pop_back();
          }
        };

        auto current_waypoint = [&]() { return parameter->shape.vertices.front(); };

        auto remain_time = [&]() {
          // TODO std::find_first_of => iter != std::end => return *iter, else infinity
          if (parameter->shape.vertices.front().time) {
            return *parameter->shape.vertices.front().time - entity_status.time;
          } else {
            return std::numeric_limits<double>::infinity();
          }
        };

        auto maximum_acceleration_currently_available = [&]() {
          return std::clamp(
            entity_status.action_status.accel.linear.x +
              behavior_parameter.dynamic_constraints.max_acceleration_rate * step_time,
            -behavior_parameter.dynamic_constraints.max_deceleration,
            +behavior_parameter.dynamic_constraints.max_acceleration);
        };

        auto minimum_acceleration_currently_available = [&]() {
          return std::clamp(
            entity_status.action_status.accel.linear.x -
              behavior_parameter.dynamic_constraints.max_deceleration_rate * step_time,
            -behavior_parameter.dynamic_constraints.max_deceleration,
            +behavior_parameter.dynamic_constraints.max_acceleration);
        };

        auto max_speed = [&]() {
          return target_speed ? *target_speed : behavior_parameter.dynamic_constraints.max_speed;
        };

        auto adjust = [&](auto suggested_speed) {
          std::cout << std::string(80, '-') << std::endl;

          std::cout << "waypoint 1/" << parameter->shape.vertices.size() << std::endl;

          if (current_waypoint().time) {
            const auto distance_to_next_waypoint =
              distance(entity_status.pose.position, current_waypoint().position.position);

            const auto estimated_time_of_arrival = distance_to_next_waypoint / suggested_speed;

            const auto desired_speed = distance_to_next_waypoint / remain_time();

            PRINT(remain_time());
            // PRINT(suggested_speed);
            PRINT(estimated_time_of_arrival);
            // PRINT(desired_speed);

            if (remain_time() < estimated_time_of_arrival) {
              std::cout << "TIME SHORTAGE => ACCELERATE" << std::endl;
              auto accelerated_speed = std::min(
                suggested_speed + maximum_acceleration_currently_available() * step_time,
                desired_speed);
              // PRINT(suggested_speed);
              // PRINT(maximum_acceleration_currently_available());
              // PRINT(maximum_acceleration_currently_available() * step_time);
              // PRINT(suggested_speed + maximum_acceleration_currently_available() * step_time);
              // PRINT(accelerated_speed);
              return accelerated_speed;
            } else if (estimated_time_of_arrival < remain_time()) {
              std::cout << "TOO FAST => DECELERATE" << std::endl;
              auto decelerated_speed = std::max(
                suggested_speed + minimum_acceleration_currently_available() * step_time,
                desired_speed);
              // PRINT(suggested_speed);
              // PRINT(minimum_acceleration_currently_available());
              // PRINT(minimum_acceleration_currently_available() * step_time);
              // PRINT(suggested_speed + minimum_acceleration_currently_available() * step_time);
              // PRINT(desired_speed);
              // PRINT(decelerated_speed);
              return decelerated_speed;
            } else {
              std::cout << "OK" << std::endl;
              return suggested_speed;
            }
          } else {
            return suggested_speed;
          }
        };

        auto steering =
          [&](auto && current_position, auto && current_target, auto && current_velocity) {
            // TODO: truncate by maximum_acceleration_currently_available() * step_time
            return normalize(current_target - current_position) *
                     behavior_parameter.dynamic_constraints.max_speed -
                   current_velocity;
          };  // vector [m/s]

        if (remain_time() <= step_time) {
          std::cout << "TIME OVER!!!" << std::endl;
          if (parameter->dynamic_constraints_ignorable) {
            std::cout << "TELEPORT!" << std::endl;
            throw std::runtime_error("TELEPORT!");
          } else {
            pop_current_waypoint();
          }
        }

        velocity +=
          steering(entity_status.pose.position, current_waypoint().position.position, velocity);

        truncate(velocity, adjust(entity_status.action_status.twist.linear.x));

        auto previous_direction = direction;

        direction.x = 0;
        direction.y = 0;
        direction.z = std::atan2(velocity.y, velocity.x);

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

        if (auto d = distance(updated_status.pose.position, current_waypoint().position.position);
            std::abs(d) <= std::numeric_limits<double>::epsilon()) {
          std::cout << "REACHED!!!" << std::endl;
          pop_current_waypoint();
        } else {
          PRINT(d);
        }

        return waypoints_exhausted() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
      }
      [[fallthrough]];

    default:
      return BT::NodeStatus::FAILURE;
  }
}
}  // namespace vehicle
}  // namespace entity_behavior
