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

#ifndef SIMULATION_API__BEHAVIOR__VEHICLE__LANE_CHANGE_ACTION_HPP_
#define SIMULATION_API__BEHAVIOR__VEHICLE__LANE_CHANGE_ACTION_HPP_

#include <simulation_api/math/hermite_curve.hpp>
#include <simulation_api/entity/vehicle_parameter.hpp>
#include <simulation_api/behavior/vehicle/vehicle_action_node.hpp>

#include <openscenario_msgs/msg/entity_status.hpp>
#include <openscenario_msgs/msg/entity_trajectory.hpp>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <boost/optional.hpp>

#include <string>
#include <memory>
#include <vector>

namespace entity_behavior
{
namespace vehicle
{
class LaneChangeAction : public entity_behavior::VehicleActionNode
{
public:
  LaneChangeAction(const std::string & name, const BT::NodeConfiguration & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = {
      BT::InputPort<std::int64_t>("to_lanelet_id")
    };
    BT::PortsList parent_ports = entity_behavior::VehicleActionNode::providedPorts();
    for (const auto & parent_port : parent_ports) {
      ports.emplace(parent_port.first, parent_port.second);
    }
    return ports;
  }
  const openscenario_msgs::msg::WaypointsArray calculateWaypoints() override;
  const std::vector<openscenario_msgs::msg::Obstacle> calculateObstacles(
    const openscenario_msgs::msg::WaypointsArray & waypoints) override;
  void getBlackBoardValues();

private:
  boost::optional<simulation_api::math::HermiteCurve> curve_;
  double current_s_;
  double target_s_;
  boost::optional<std::int64_t> to_lanelet_id_;
};
}      // namespace vehicle
}  // namespace entity_behavior

#endif  // SIMULATION_API__BEHAVIOR__VEHICLE__LANE_CHANGE_ACTION_HPP_
