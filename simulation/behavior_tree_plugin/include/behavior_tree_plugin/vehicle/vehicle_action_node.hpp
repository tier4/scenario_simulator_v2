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

#ifndef BEHAVIOR_TREE_PLUGIN__VEHICLE__VEHICLE_ACTION_NODE_HPP_
#define BEHAVIOR_TREE_PLUGIN__VEHICLE__VEHICLE_ACTION_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>

#include <behavior_tree_plugin/action_node.hpp>
#include <memory>
#include <string>
#include <traffic_simulator/helper/stop_watch.hpp>
#include <traffic_simulator/math/catmull_rom_spline.hpp>
#include <traffic_simulator/math/catmull_rom_subspline.hpp>
#include <traffic_simulator_msgs/msg/driver_model.hpp>
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
      BT::InputPort<traffic_simulator_msgs::msg::DriverModel>("driver_model"),
      BT::InputPort<traffic_simulator_msgs::msg::VehicleParameters>("vehicle_parameters"),
      BT::InputPort<std::shared_ptr<traffic_simulator::math::CatmullRomSpline>>(
        "reference_trajectory")};
    BT::PortsList parent_ports = entity_behavior::ActionNode::providedPorts();
    for (const auto & parent_port : parent_ports) {
      ports.emplace(parent_port.first, parent_port.second);
    }
    return ports;
  }
  traffic_simulator_msgs::msg::EntityStatus calculateEntityStatusUpdated(double target_speed);
  traffic_simulator_msgs::msg::EntityStatus calculateEntityStatusUpdated(
    double target_speed, const std::vector<std::int64_t> & following_lanelets);
  traffic_simulator_msgs::msg::EntityStatus calculateEntityStatusUpdatedInWorldFrame(
    double target_speed);
  virtual const traffic_simulator_msgs::msg::WaypointsArray calculateWaypoints() = 0;
  virtual const boost::optional<traffic_simulator_msgs::msg::Obstacle> calculateObstacle(
    const traffic_simulator_msgs::msg::WaypointsArray & waypoints) = 0;

protected:
  traffic_simulator_msgs::msg::DriverModel driver_model;
  traffic_simulator_msgs::msg::VehicleParameters vehicle_parameters;
  std::shared_ptr<traffic_simulator::math::CatmullRomSpline> reference_trajectory;
  std::unique_ptr<traffic_simulator::math::CatmullRomSubspline> trajectory;
};
}  // namespace entity_behavior

#endif  // BEHAVIOR_TREE_PLUGIN__VEHICLE__VEHICLE_ACTION_NODE_HPP_
