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

  for (auto && vertex : parameter.shape.vertices) {
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
  -> const boost::optional<traffic_simulator_msgs::msg::Obstacle>
{
  return boost::none;
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

#define DEFINE_VECTOR3_VS_VECTOR3_BINARY_OPERATOR(OPERATOR)                      \
  template <                                                                     \
    typename T, typename U,                                                      \
    std::enable_if_t<std::conjunction_v<is_vector3<T>, is_vector3<U>>, int> = 0> \
  auto operator OPERATOR(const T & a, const U & b)                               \
  {                                                                              \
    geometry_msgs::msg::Vector3 v;                                               \
    v.x = a.x OPERATOR b.x;                                                      \
    v.y = a.y OPERATOR b.y;                                                      \
    v.z = a.z OPERATOR b.z;                                                      \
    return v;                                                                    \
  }                                                                              \
  static_assert(true)

DEFINE_VECTOR3_VS_VECTOR3_BINARY_OPERATOR(+);
DEFINE_VECTOR3_VS_VECTOR3_BINARY_OPERATOR(-);
DEFINE_VECTOR3_VS_VECTOR3_BINARY_OPERATOR(*);
DEFINE_VECTOR3_VS_VECTOR3_BINARY_OPERATOR(/);

#define DEFINE_VECTOR3_VS_VECTOR3_COMPOUND_ASSIGNMENT_OPERATOR(OPERATOR)         \
  template <                                                                     \
    typename T, typename U,                                                      \
    std::enable_if_t<std::conjunction_v<is_vector3<T>, is_vector3<U>>, int> = 0> \
  auto operator OPERATOR(T & a, const U & b)->decltype(auto)                     \
  {                                                                              \
    a.x OPERATOR b.x;                                                            \
    a.y OPERATOR b.y;                                                            \
    a.z OPERATOR b.z;                                                            \
    return a;                                                                    \
  }                                                                              \
  static_assert(true)

DEFINE_VECTOR3_VS_VECTOR3_COMPOUND_ASSIGNMENT_OPERATOR(+=);
DEFINE_VECTOR3_VS_VECTOR3_COMPOUND_ASSIGNMENT_OPERATOR(-=);
DEFINE_VECTOR3_VS_VECTOR3_COMPOUND_ASSIGNMENT_OPERATOR(*=);
DEFINE_VECTOR3_VS_VECTOR3_COMPOUND_ASSIGNMENT_OPERATOR(/=);

#define DEFINE_VECTOR3_VS_SCALAR_BINARY_OPERATOR(OPERATOR)                           \
  template <                                                                         \
    typename T, typename U,                                                          \
    std::enable_if_t<std::conjunction_v<is_vector3<T>, std::is_scalar<U>>, int> = 0> \
  auto operator OPERATOR(const T & a, const U & b)                                   \
  {                                                                                  \
    geometry_msgs::msg::Vector3 v;                                                   \
    v.x = a.x OPERATOR b;                                                            \
    v.y = a.y OPERATOR b;                                                            \
    v.z = a.z OPERATOR b;                                                            \
    return v;                                                                        \
  }                                                                                  \
  static_assert(true)

DEFINE_VECTOR3_VS_SCALAR_BINARY_OPERATOR(+);
DEFINE_VECTOR3_VS_SCALAR_BINARY_OPERATOR(-);
DEFINE_VECTOR3_VS_SCALAR_BINARY_OPERATOR(*);
DEFINE_VECTOR3_VS_SCALAR_BINARY_OPERATOR(/);

#define DEFINE_VECTOR3_VS_SCALAR_COMPOUND_ASSIGNMENT_OPERATOR(OPERATOR)              \
  template <                                                                         \
    typename T, typename U,                                                          \
    std::enable_if_t<std::conjunction_v<is_vector3<T>, std::is_scalar<U>>, int> = 0> \
  auto operator OPERATOR(T & a, const U & b)                                         \
  {                                                                                  \
    a.x OPERATOR b;                                                                  \
    a.y OPERATOR b;                                                                  \
    a.z OPERATOR b;                                                                  \
    return a;                                                                        \
  }                                                                                  \
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
        getInput<Parameter>("polyline_trajectory_parameter", parameter) and
        getInput<decltype(target_speed)>("target_speed", target_speed) and
        not parameter.shape.vertices.empty()) {
        // PRINT(parameter.initial_distance_offset);
        // PRINT(parameter.dynamic_constraints_ignorable);
        // PRINT(parameter.closed);

        auto updated_status = entity_status;

        auto current_max_acceleration = [&]() {
          return std::clamp(
            entity_status.action_status.accel.linear.x +
              behavior_parameter.dynamic_constraints.max_acceleration_rate * step_time,
            0.0, behavior_parameter.dynamic_constraints.max_acceleration);
        };

        auto current_max_speed = [&]() {
          return std::clamp(
            entity_status.action_status.twist.linear.x + current_max_acceleration() * step_time,
            0.0, target_speed ? *target_speed : behavior_parameter.dynamic_constraints.max_speed);
        };  // scalar [m/s]

        auto steering =
          [&](auto && current_position, auto && current_target, auto && current_velocity) {
            // TODO: tuncate by current_max_acceleration() * step_time
            return normalize(current_target - current_position) *
                     behavior_parameter.dynamic_constraints.max_speed -
                   current_velocity;
          };  // vector [m/s]

        truncate(
          velocity += steering(
            entity_status.pose.position,
            parameter.shape.vertices.at(current_waypoint_index).position.position, velocity),
          current_max_speed());

        auto previous_direction = direction;

        direction.x = 0;
        direction.y = 0;
        direction.z = std::atan2(velocity.y, velocity.x);

        updated_status.action_status.twist.linear.x = norm(velocity);
        updated_status.action_status.twist.linear.y = 0;
        updated_status.action_status.twist.linear.z = 0;

        updated_status.action_status.twist.angular.x = 0;
        updated_status.action_status.twist.angular.y = 0;
        updated_status.action_status.twist.angular.z =
          (direction.z - previous_direction.z) / step_time;

        updated_status.action_status.accel.linear =
          updated_status.action_status.twist.linear / step_time;
        updated_status.action_status.accel.angular =
          updated_status.action_status.twist.angular / step_time;

        updated_status.pose.position += velocity * step_time;

        updated_status.pose.orientation =
          quaternion_operation::convertEulerAngleToQuaternion(direction);

        updated_status.time = entity_status.time + step_time;

        updated_status.lanelet_pose_valid = false;

        setOutput("updated_status", updated_status);
        setOutput("waypoints", calculateWaypoints());
        setOutput("obstacle", calculateObstacle(calculateWaypoints()));

        if (auto d = distance(
              updated_status.pose.position,
              parameter.shape.vertices.at(current_waypoint_index).position.position);
            d < 1) {
          ++current_waypoint_index;
        }

        return current_waypoint_index == parameter.shape.vertices.size() ? BT::NodeStatus::SUCCESS
                                                                         : BT::NodeStatus::RUNNING;
      }
      [[fallthrough]];

    default:
      return BT::NodeStatus::FAILURE;
  }
}
}  // namespace vehicle
}  // namespace entity_behavior
