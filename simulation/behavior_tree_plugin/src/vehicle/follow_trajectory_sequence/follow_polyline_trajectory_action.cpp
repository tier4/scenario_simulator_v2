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
  LINE();
  return traffic_simulator_msgs::msg::WaypointsArray();
}

auto FollowPolylineTrajectoryAction::calculateObstacle(
  const traffic_simulator_msgs::msg::WaypointsArray &)
  -> const boost::optional<traffic_simulator_msgs::msg::Obstacle>
{
  LINE();
  return boost::none;
}

auto FollowPolylineTrajectoryAction::tick() -> BT::NodeStatus
{
  getBlackBoardValues();

  if (request == traffic_simulator::behavior::Request::FOLLOW_POLYLINE_TRAJECTORY) {
    auto updated_status = entity_status;

    setOutput("updated_status", updated_status);
    setOutput("waypoints", calculateWaypoints());
    setOutput("obstacle", calculateObstacle(calculateWaypoints()));

    return BT::NodeStatus::RUNNING;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}
}  // namespace vehicle
}  // namespace entity_behavior
