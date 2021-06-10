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

#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__VEHICLE__VEHICLE_ACTION_NODE_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__VEHICLE__VEHICLE_ACTION_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>

#include <memory>
#include <openscenario_msgs/msg/driver_model.hpp>
#include <openscenario_msgs/msg/entity_trajectory.hpp>
#include <openscenario_msgs/msg/obstacle.hpp>
#include <openscenario_msgs/msg/vehicle_parameters.hpp>
#include <openscenario_msgs/msg/waypoints_array.hpp>
#include <string>
#include <traffic_simulator/behavior/action_node.hpp>
#include <traffic_simulator/entity/vehicle_parameter.hpp>
#include <traffic_simulator/helper/stop_watch.hpp>
#include <traffic_simulator/math/catmull_rom_spline.hpp>
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
      BT::InputPort<openscenario_msgs::msg::DriverModel>("driver_model"),
      BT::InputPort<openscenario_msgs::msg::VehicleParameters>("vehicle_parameters")};
    BT::PortsList parent_ports = entity_behavior::ActionNode::providedPorts();
    for (const auto & parent_port : parent_ports) {
      ports.emplace(parent_port.first, parent_port.second);
    }
    return ports;
  }
  openscenario_msgs::msg::VehicleParameters vehicle_parameters;
  openscenario_msgs::msg::EntityStatus calculateEntityStatusUpdated(double target_speed);
  openscenario_msgs::msg::EntityStatus calculateEntityStatusUpdated(
    double target_speed, const std::vector<std::int64_t> & following_lanelets);
  openscenario_msgs::msg::EntityStatus calculateEntityStatusUpdatedInWorldFrame(
    double target_speed);
  virtual const openscenario_msgs::msg::WaypointsArray calculateWaypoints() = 0;
  virtual const boost::optional<openscenario_msgs::msg::Obstacle> calculateObstacle(
    const openscenario_msgs::msg::WaypointsArray & waypoints) = 0;
  openscenario_msgs::msg::DriverModel driver_model;
};
}  // namespace entity_behavior

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__VEHICLE__VEHICLE_ACTION_NODE_HPP_
