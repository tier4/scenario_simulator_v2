// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__VEHICLE__FOLLOW_LANE_SEQUENCE__FOLLOW_FRONT_ENTITY_ACTION_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__VEHICLE__FOLLOW_LANE_SEQUENCE__FOLLOW_FRONT_ENTITY_ACTION_HPP_

#include <string>
#include <traffic_simulator/behavior/vehicle/vehicle_action_node.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <vector>

namespace entity_behavior
{
namespace vehicle
{
namespace follow_lane_sequence
{
class FollowFrontEntityAction : public entity_behavior::VehicleActionNode
{
public:
  FollowFrontEntityAction(const std::string & name, const BT::NodeConfiguration & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = {};
    BT::PortsList parent_ports = entity_behavior::VehicleActionNode::providedPorts();
    for (const auto & parent_port : parent_ports) {
      ports.emplace(parent_port.first, parent_port.second);
    }
    return ports;
  }
  const openscenario_msgs::msg::WaypointsArray calculateWaypoints() override;
  const boost::optional<openscenario_msgs::msg::Obstacle> calculateObstacle(
    const openscenario_msgs::msg::WaypointsArray & waypoints) override;

private:
  boost::optional<double> distance_to_front_entity_;
};
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__VEHICLE__FOLLOW_LANE_SEQUENCE__FOLLOW_FRONT_ENTITY_ACTION_HPP_
