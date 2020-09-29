// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef ENTITY_BEHAVIOR__PEDESTRIAN__ACQUIRE_POSITION_ACTION_HPP
#define ENTITY_BEHAVIOR__PEDESTRIAN__ACQUIRE_POSITION_ACTION_HPP

#include <simulation_controller/entity/pedestrian_parameter.hpp>
#include <simulation_controller/entity/entity_status.hpp>
#include <simulation_controller/behavior/action_node.hpp>
#include <simulation_controller/hdmap_utils/hdmap_utils.hpp>

#include <geometry_msgs/msg/point.hpp>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <boost/optional.hpp>

namespace entity_behavior
{
namespace pedestrian
{
class AcquirePositionAction : public entity_behavior::ActionNode
{
public:
  AcquirePositionAction(const std::string & name, const BT::NodeConfiguration & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<std::string>("request"),
        BT::InputPort<std::shared_ptr<hdmap_utils::HdMapUtils>>("hdmap_utils"),
        BT::InputPort<simulation_controller::entity::EntityStatus>("entity_status"),
        BT::InputPort<double>("current_time"),
        BT::InputPort<double>("step_time"),
        BT::InputPort<boost::optional<double>>("target_speed"),
        BT::InputPort<std::shared_ptr<simulation_controller::entity::PedestrianParameters>>(
          "pedestrian_parameters"),
        BT::OutputPort<std::vector<geometry_msgs::msg::Point>>("trajectory"),
        BT::OutputPort<simulation_controller::entity::EntityStatus>("updated_status"),
        BT::OutputPort<std::string>("request"),

        BT::InputPort<simulation_controller::entity::EntityStatus>("target_status")
      };
  }

private:
  boost::optional<simulation_controller::entity::EntityStatus> target_status_;
  std::vector<geometry_msgs::msg::Point> following_trajectory_;
  boost::optional<std::vector<int>> route_;
};
}      // namespace pedestrian
}  // namespace entity_behavior

#endif  // ENTITY_BEHAVIOR__PEDESTRIAN__ACQUIRE_POSITION_ACTION_HPP
