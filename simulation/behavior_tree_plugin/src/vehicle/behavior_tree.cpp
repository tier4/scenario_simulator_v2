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

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behavior_tree_plugin/vehicle/behavior_tree.hpp>
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/follow_front_entity_action.hpp>
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/follow_lane_action.hpp>
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/move_backward_action.hpp>
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/stop_at_crossing_entity_action.hpp>
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/stop_at_stop_line_action.hpp>
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/stop_at_traffic_light_action.hpp>
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/yield_action.hpp>
#include <behavior_tree_plugin/vehicle/follow_trajectory_sequence/follow_polyline_trajectory_action.hpp>
#include <behavior_tree_plugin/vehicle/lane_change_action.hpp>
#include <iostream>
#include <pugixml.hpp>
#include <sstream>
#include <string>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <utility>

namespace entity_behavior
{
void VehicleBehaviorTree::configure(const rclcpp::Logger & logger)
{
  factory_.registerNodeType<vehicle::follow_lane_sequence::FollowLaneAction>("FollowLane");
  factory_.registerNodeType<vehicle::follow_lane_sequence::FollowFrontEntityAction>(
    "FollowFrontEntity");
  factory_.registerNodeType<vehicle::follow_lane_sequence::StopAtCrossingEntityAction>(
    "StopAtCrossingEntity");
  factory_.registerNodeType<vehicle::follow_lane_sequence::StopAtStopLineAction>("StopAtStopLine");
  factory_.registerNodeType<vehicle::follow_lane_sequence::StopAtTrafficLightAction>(
    "StopAtTrafficLight");
  factory_.registerNodeType<vehicle::follow_lane_sequence::YieldAction>("Yield");
  factory_.registerNodeType<vehicle::follow_lane_sequence::MoveBackwardAction>("MoveBackward");
  factory_.registerNodeType<vehicle::FollowPolylineTrajectoryAction>("FollowPolylineTrajectory");
  factory_.registerNodeType<vehicle::LaneChangeAction>("LaneChange");

  tree_ = createBehaviorTree(
    ament_index_cpp::get_package_share_directory("behavior_tree_plugin") +
    "/config/vehicle_entity_behavior.xml");

  logging_event_ptr_ =
    std::make_unique<behavior_tree_plugin::LoggingEvent>(tree_.rootNode(), logger);

  reset_request_event_ptr_ = std::make_unique<behavior_tree_plugin::ResetRequestEvent>(
    tree_.rootNode(), [&]() { return getRequest(); },
    [&]() { setRequest(traffic_simulator::behavior::Request::NONE); });

  setRequest(traffic_simulator::behavior::Request::NONE);
}

auto VehicleBehaviorTree::createBehaviorTree(const std::string & format_path) -> BT::Tree
{
  auto xml_doc = pugi::xml_document();
  xml_doc.load_file(format_path.c_str());

  class XMLTreeWalker : public pugi::xml_tree_walker
  {
  public:
    explicit XMLTreeWalker(const BT::TreeNodeManifest & manifest) : manifest_(manifest) {}

  private:
    bool for_each(pugi::xml_node & node) final
    {
      if (node.name() == manifest_.registration_ID) {
        for (const auto & [port, info] : manifest_.ports) {
          node.append_attribute(port.c_str()) = std::string("{" + port + "}").c_str();
        }
      }
      return true;
    }

    const BT::TreeNodeManifest & manifest_;
  };

  for (const auto & [id, manifest] : factory_.manifests()) {
    if (factory_.builtinNodes().count(id) == 0) {
      auto walker = XMLTreeWalker(manifest);
      xml_doc.traverse(walker);
    }
  }

  auto xml_str = std::stringstream();
  xml_doc.save(xml_str);
  return factory_.createTreeFromText(xml_str.str());
}

auto VehicleBehaviorTree::getBehaviorParameter() -> traffic_simulator_msgs::msg::BehaviorParameter
{
  return tree_.rootBlackboard()->get<traffic_simulator_msgs::msg::BehaviorParameter>(
    getBehaviorParameterKey());
}

auto VehicleBehaviorTree::setBehaviorParameter(
  const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter) -> void
{
  const auto vehicle_parameters = getVehicleParameters();

  auto clamp = [&](const auto & behavior_parameter) {
    auto result = behavior_parameter;

    result.dynamic_constraints.max_acceleration = std::clamp(
      result.dynamic_constraints.max_acceleration, 0.0,
      vehicle_parameters.performance.max_acceleration);

    result.dynamic_constraints.max_acceleration_rate = std::clamp(
      result.dynamic_constraints.max_acceleration_rate, 0.0,
      vehicle_parameters.performance.max_acceleration_rate);

    result.dynamic_constraints.max_deceleration = std::clamp(
      result.dynamic_constraints.max_deceleration, 0.0,
      vehicle_parameters.performance.max_deceleration);

    result.dynamic_constraints.max_deceleration_rate = std::clamp(
      result.dynamic_constraints.max_deceleration_rate, 0.0,
      vehicle_parameters.performance.max_deceleration_rate);

    result.dynamic_constraints.max_speed = std::clamp(
      result.dynamic_constraints.max_speed, 0.0, vehicle_parameters.performance.max_speed);
    return result;
  };

  tree_.rootBlackboard()->set<traffic_simulator_msgs::msg::BehaviorParameter>(
    getBehaviorParameterKey(), clamp(behavior_parameter));
}

const std::string & VehicleBehaviorTree::getCurrentAction() const
{
  return logging_event_ptr_->getCurrentAction();
}

auto VehicleBehaviorTree::update(const double current_time, const double step_time) -> void
{
  tickOnce(current_time, step_time);
  while (getCurrentAction() == "root") {
    tickOnce(current_time, step_time);
  }
}

auto VehicleBehaviorTree::tickOnce(const double current_time, const double step_time)
  -> BT::NodeStatus
{
  setCurrentTime(current_time);
  setStepTime(step_time);
  return tree_.rootNode()->executeTick();
}
}  // namespace entity_behavior

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(entity_behavior::VehicleBehaviorTree, entity_behavior::BehaviorPluginBase)
