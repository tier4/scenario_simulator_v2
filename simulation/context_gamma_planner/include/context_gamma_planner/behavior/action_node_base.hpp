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

#ifndef CONTEXT_GAMMA_PLANNER__ACTION_NODE_HPP_
#define CONTEXT_GAMMA_PLANNER__ACTION_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>

#include <memory>
#include <string>
#include <traffic_simulator/behavior/behavior_plugin_base.hpp>
#include <traffic_simulator/data_type/behavior.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights_base.hpp>
#include <traffic_simulator_msgs/msg/obstacle.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <vector>

namespace context_gamma_planner
{
class ActionNodeBase : public BT::ActionNodeBase
{
public:
  ActionNodeBase(const std::string & name, const BT::NodeConfiguration & config);
  ~ActionNodeBase() override = default;
  void halt() final { setStatus(BT::NodeStatus::IDLE); }

  static BT::PortsList providedPorts()
  {
    return {// clang-format off
      BT::InputPort<traffic_simulator::behavior::Request>("request"),
      BT::InputPort<std::shared_ptr<hdmap_utils::HdMapUtils>>("hdmap_utils"),
      BT::InputPort<std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus>>("canonicalized_entity_status"),
      BT::InputPort<double>("current_time"),
      BT::InputPort<double>("step_time"),
      BT::InputPort<entity_behavior::EntityStatusDict>("other_entity_status"),
      BT::InputPort<std::vector<geometry_msgs::msg::Pose>>("goal_poses"),
      BT::InputPort<std::vector<lanelet::Id>>("route_lanelets"),
      BT::InputPort<std::optional<double>>("target_speed"),
      BT::OutputPort<std::optional<double>>("planning_speed"),
      BT::OutputPort<geometry_msgs::msg::Point>("next_goal"),
      BT::OutputPort<std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus>>("updated_status"),
      BT::OutputPort<traffic_simulator::behavior::Request>("request"),
      BT::InputPort<entity_behavior::EntityStatusDict>("other_entity_status"),
      BT::InputPort<std::vector<lanelet::Id>>("route_lanelets"),
      BT::InputPort<std::shared_ptr<traffic_simulator::TrafficLightsBase>>("traffic_lights"),
      BT::OutputPort<std::shared_ptr<hdmap_utils::HdMapUtils>>("hdmap_utils"),
      BT::OutputPort<std::optional<traffic_simulator_msgs::msg::Obstacle>>("obstacle"),
      BT::OutputPort<traffic_simulator_msgs::msg::WaypointsArray>("waypoints")};
    // clang-format on
  }

  virtual auto getBlackBoardValues() -> void;

protected:
  traffic_simulator::behavior::Request request;
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils;
  std::shared_ptr<traffic_simulator::TrafficLightsBase> traffic_lights;
  std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> entity_status;
  double current_time;
  double step_time;
  std::optional<double> target_speed;
  std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> updated_status;
  entity_behavior::EntityStatusDict other_entity_status;
  std::vector<lanelet::Id> route_lanelets;
};
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__ACTION_NODE_HPP_
