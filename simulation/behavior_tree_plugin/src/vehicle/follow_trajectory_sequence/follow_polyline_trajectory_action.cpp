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
  return std::forward<decltype(ports)>(ports);
}

#define DEFINE_VECTOR3_VS_VECTOR3_BINARY_OPERATOR(OPERATOR)                       \
  auto operator OPERATOR(                                                         \
    const geometry_msgs::msg::Vector3 & a, const geometry_msgs::msg::Vector3 & b) \
  {                                                                               \
    geometry_msgs::msg::Vector3 v;                                                \
    v.x = a.x OPERATOR b.x;                                                       \
    v.y = a.y OPERATOR b.y;                                                       \
    v.z = a.z OPERATOR b.z;                                                       \
    return v;                                                                     \
  }                                                                               \
  static_assert(true)

DEFINE_VECTOR3_VS_VECTOR3_BINARY_OPERATOR(+);
DEFINE_VECTOR3_VS_VECTOR3_BINARY_OPERATOR(-);
DEFINE_VECTOR3_VS_VECTOR3_BINARY_OPERATOR(*);
DEFINE_VECTOR3_VS_VECTOR3_BINARY_OPERATOR(/);

#define DEFINE_VECTOR3_VS_DOUBLE_BINARY_OPERATOR(OPERATOR)                \
  auto operator OPERATOR(const geometry_msgs::msg::Vector3 & a, double b) \
  {                                                                       \
    geometry_msgs::msg::Vector3 v;                                        \
    v.x = a.x OPERATOR b;                                                 \
    v.y = a.y OPERATOR b;                                                 \
    v.z = a.z OPERATOR b;                                                 \
    return v;                                                             \
  }                                                                       \
  static_assert(true)

DEFINE_VECTOR3_VS_DOUBLE_BINARY_OPERATOR(+);
DEFINE_VECTOR3_VS_DOUBLE_BINARY_OPERATOR(-);
DEFINE_VECTOR3_VS_DOUBLE_BINARY_OPERATOR(*);
DEFINE_VECTOR3_VS_DOUBLE_BINARY_OPERATOR(/);

auto vector3(const geometry_msgs::msg::Point & point)
{
  geometry_msgs::msg::Vector3 v;
  v.x = point.x;
  v.y = point.y;
  v.z = point.z;
  return v;
}

auto FollowPolylineTrajectoryAction::tick() -> BT::NodeStatus
{
  getBlackBoardValues();

  auto distance = [](const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to) {
    return std::hypot(
      to.position.x - from.position.x, to.position.y - from.position.y,
      to.position.z - from.position.z);
  };

  auto norm = [](const geometry_msgs::msg::Vector3 & v) { return std::hypot(v.x, v.y, v.z); };

  auto normalize = [&](const geometry_msgs::msg::Vector3 & v) { return v / norm(v); };

  auto speed = [&]() { return behavior_parameter.dynamic_constraints.max_speed / 10; };

  auto make_steering =
    [&](const auto & position, const auto & target, const auto & current_velocity) {
      return normalize(vector3(target) - vector3(position)) * speed() - current_velocity;  // [m/s]
    };

  auto truncate = [](const geometry_msgs::msg::Vector3 & steering, double max_force) {
    geometry_msgs::msg::Vector3 v;
    v.x = std::min(steering.x, max_force);
    v.y = std::min(steering.y, max_force);
    v.z = std::min(steering.z, max_force);
    return v;
  };

  switch (request) {
    case traffic_simulator::behavior::Request::FOLLOW_POLYLINE_TRAJECTORY:
      if (
        getInput<Parameter>("polyline_trajectory_parameter", parameter) and
        not parameter.shape.vertices.empty()) {
        // PRINT(parameter.initial_distance_offset);
        // PRINT(parameter.dynamic_constraints_ignorable);
        // PRINT(parameter.closed);

        auto updated_status = entity_status;

        if (parameter.shape.vertices.front().time) {
          // TODO
        } else {
          auto steering = make_steering(
            entity_status.pose.position,
            parameter.shape.vertices.at(current_waypoint_index).position.position,
            entity_status.action_status.twist.linear);  // [m/s]

          updated_status.action_status.twist.linear.x += steering.x;  // [m/s]
          updated_status.action_status.twist.linear.y += steering.y;
          updated_status.action_status.twist.linear.z += steering.z;

          updated_status.action_status.twist.angular.x = 0;
          updated_status.action_status.twist.angular.y = 0;
          updated_status.action_status.twist.angular.z = std::atan2(
            updated_status.action_status.twist.linear.y,
            updated_status.action_status.twist.linear.x);

          updated_status.pose.position.x +=
            updated_status.action_status.twist.linear.x * step_time;  // [m/s] * [s] = [m]
          updated_status.pose.position.y += updated_status.action_status.twist.linear.y * step_time;
          updated_status.pose.position.z += updated_status.action_status.twist.linear.z * step_time;

          updated_status.pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(
            updated_status.action_status.twist.angular);
        }

        setOutput("updated_status", updated_status);
        setOutput("waypoints", calculateWaypoints());
        setOutput("obstacle", calculateObstacle(calculateWaypoints()));

        if (auto d = distance(
              updated_status.pose, parameter.shape.vertices.at(current_waypoint_index).position);
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
