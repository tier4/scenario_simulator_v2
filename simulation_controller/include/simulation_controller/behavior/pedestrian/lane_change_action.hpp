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

#ifndef SIMULATION_CONTROLLER__BEHAVIOR__PEDESTRIAN__LANE_CHANGE_ACTION_HPP_
#define SIMULATION_CONTROLLER__BEHAVIOR__PEDESTRIAN__LANE_CHANGE_ACTION_HPP_

#include <simulation_controller/math/hermite_curve.hpp>
#include <simulation_controller/entity/pedestrian_parameter.hpp>
#include <simulation_controller/entity/entity_status.hpp>
#include <simulation_controller/behavior/action_node.hpp>
#include <simulation_controller/hdmap_utils/hdmap_utils.hpp>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <boost/optional.hpp>

#include <string>
#include <memory>
#include <vector>

namespace entity_behavior
{
namespace pedestrian
{
struct LaneChangeParameter
{
  int to_lanelet_id;
};

class LaneChangeAction : public entity_behavior::ActionNode
{
public:
  LaneChangeAction(const std::string & name, const BT::NodeConfiguration & config);
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
        BT::InputPort<std::shared_ptr<simulation_controller::entity::PedestrianParameters>>(
          "pedestrian_parameters"),
        BT::OutputPort<std::vector<geometry_msgs::msg::Point>>("trajectory"),
        BT::OutputPort<simulation_controller::entity::EntityStatus>("updated_status"),

        BT::InputPort<LaneChangeParameter>("lane_change_params")
      };
  }

private:
  boost::optional<simulation_controller::math::HermiteCurve> curve_;
  double current_s_;
  double target_s_;
};
}      // namespace pedestrian
}  // namespace entity_behavior

#endif  // SIMULATION_CONTROLLER__BEHAVIOR__PEDESTRIAN__LANE_CHANGE_ACTION_HPP_
