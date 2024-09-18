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
#include <behavior_tree_plugin/pedestrian/behavior_tree.hpp>
#include <behavior_tree_plugin/pedestrian/follow_trajectory_sequence/follow_polyline_trajectory_action.hpp>
#include <iostream>
#include <memory>
#include <pugixml.hpp>
#include <string>
#include <utility>

namespace entity_behavior
{
void PedestrianBehaviorTree::configure(const rclcpp::Logger & logger)
{
  namespace pedestrian = entity_behavior::pedestrian;
  factory_.registerNodeType<pedestrian::FollowLaneAction>("FollowLane");
  factory_.registerNodeType<pedestrian::WalkStraightAction>("WalkStraightAction");
  factory_.registerNodeType<pedestrian::FollowPolylineTrajectoryAction>("FollowPolylineTrajectory");

  auto base_path = ament_index_cpp::get_package_share_directory("behavior_tree_plugin");
  auto format_path = base_path + "/config/pedestrian_entity_behavior.xml";
  tree_ = createBehaviorTree(format_path);
  logging_event_ptr_ =
    std::make_unique<behavior_tree_plugin::LoggingEvent>(tree_.rootNode(), logger);
  reset_request_event_ptr_ = std::make_unique<behavior_tree_plugin::ResetRequestEvent>(
    tree_.rootNode(), [&]() { return getRequest(); },
    [&]() { setRequest(traffic_simulator::behavior::Request::NONE); });
  setRequest(traffic_simulator::behavior::Request::NONE);
}

auto PedestrianBehaviorTree::createBehaviorTree(const std::string & format_path) -> BT::Tree
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

const std::string & PedestrianBehaviorTree::getCurrentAction() const
{
  return logging_event_ptr_->getCurrentAction();
}

auto PedestrianBehaviorTree::update(const double current_time, const double step_time) -> void
{
  tickOnce(current_time, step_time);
  while (getCurrentAction() == "root") {
    tickOnce(current_time, step_time);
  }
}

auto PedestrianBehaviorTree::tickOnce(const double current_time, const double step_time)
  -> BT::NodeStatus
{
  setCurrentTime(current_time);
  setStepTime(step_time);
  return tree_.rootNode()->executeTick();
}
}  // namespace entity_behavior

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(entity_behavior::PedestrianBehaviorTree, entity_behavior::BehaviorPluginBase)
