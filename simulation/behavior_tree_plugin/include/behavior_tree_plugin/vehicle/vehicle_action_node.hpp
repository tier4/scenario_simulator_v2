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

#ifndef BEHAVIOR_TREE_PLUGIN__VEHICLE__VEHICLE_ACTION_NODE_HPP_
#define BEHAVIOR_TREE_PLUGIN__VEHICLE__VEHICLE_ACTION_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>

#include <behavior_tree_plugin/action_node.hpp>
#include <geometry/spline/catmull_rom_spline.hpp>
#include <geometry/spline/catmull_rom_subspline.hpp>
#include <memory>
#include <optional>
#include <string>
#include <traffic_simulator/helper/stop_watch.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <traffic_simulator_msgs/msg/obstacle.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <vector>

namespace entity_behavior
{
class VehicleActionNode : public ActionNode
{
public:
  VehicleActionNode(const std::string & name, const BT::NodeConfiguration & config);
  ~VehicleActionNode() override = default;
  void getBlackBoardValues();
  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = {
      // clang-format off
      BT::InputPort<std::shared_ptr<math::geometry::CatmullRomSpline>>("reference_trajectory"),
      BT::InputPort<traffic_simulator_msgs::msg::BehaviorParameter>("behavior_parameter"),
      BT::InputPort<traffic_simulator_msgs::msg::VehicleParameters>("vehicle_parameters"),
      // clang-format on
    };
    BT::PortsList parent_ports = entity_behavior::ActionNode::providedPorts();
    for (const auto & parent_port : parent_ports) {
      ports.emplace(parent_port.first, parent_port.second);
    }
    return ports;
  }
  auto calculateUpdatedEntityStatus(double target_speed) const -> traffic_simulator::EntityStatus;
  auto calculateUpdatedEntityStatusInWorldFrame(double target_speed) const
    -> traffic_simulator::EntityStatus;
  virtual const traffic_simulator_msgs::msg::WaypointsArray calculateWaypoints() = 0;
  virtual const std::optional<traffic_simulator_msgs::msg::Obstacle> calculateObstacle(
    const traffic_simulator_msgs::msg::WaypointsArray & waypoints) = 0;

protected:
  traffic_simulator_msgs::msg::BehaviorParameter behavior_parameter;
  traffic_simulator_msgs::msg::VehicleParameters vehicle_parameters;
  std::shared_ptr<math::geometry::CatmullRomSpline> reference_trajectory;
  std::unique_ptr<math::geometry::CatmullRomSubspline> trajectory;
};
}  // namespace entity_behavior

#endif  // BEHAVIOR_TREE_PLUGIN__VEHICLE__VEHICLE_ACTION_NODE_HPP_
