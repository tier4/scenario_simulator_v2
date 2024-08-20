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

#ifndef OPENSCENARIO_VISUALIZATION_PARAMETERS_HPP_
#define OPENSCENARIO_VISUALIZATION_PARAMETERS_HPP_

#include <yaml-cpp/yaml.h>

#include <memory>
#include <nlohmann/json.hpp>

#ifndef Q_MOC_RUN
#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <openscenario_interpreter_msgs/msg/context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/validate_floats.hpp>
#include <std_msgs/msg/string.hpp>

#include "jsk_overlay_utils.hpp"
#endif

namespace openscenario_visualization
{
// struct Condition
// {
//   std::string current_evaluation;
//   std::string current_value;
//   std::string type;
// };

// struct ConditionGroup
// {
//   std::vector<Condition> conditions;
// };

// struct ConditionGroups
// {
//   std::string groups_name;
//   std::vector<ConditionGroup> condition_groups;
// };

// using nlohmann::json;
// using openscenario_interpreter_msgs::msg::Context;
// using ConditionGroupsCollection = std::vector<ConditionGroups>;

class VisualizationParametersDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  VisualizationParametersDisplay();
  virtual ~VisualizationParametersDisplay();

  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;
  void subscribe();
  void unsubscribe();

private Q_SLOTS:
  void updateTopic();
  void updateVisualization();

protected:
  void processMessage(const std_msgs::msg::String::ConstSharedPtr msg_ptr);
  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz_common::properties::ColorProperty * property_text_color_;
  rviz_common::properties::IntProperty * property_left_;
  rviz_common::properties::IntProperty * property_top_;
  rviz_common::properties::IntProperty * property_length_;
  rviz_common::properties::IntProperty * property_width_;
  rviz_common::properties::StringProperty * property_topic_name_;
  rviz_common::properties::FloatProperty * property_value_scale_;

private:
  //void loadConditionGroups(const std_msgs::msg::String::ConstSharedPtr msg_ptr);
  //void processStory(const YAML::Node & story);
  //void processManeuver(const YAML::Node & maneuver);
  //void processEvent(const YAML::Node & event);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr simulation_context_sub_;
  std_msgs::msg::String::ConstSharedPtr last_msg_ptr_;
  //std::shared_ptr<ConditionGroupsCollection> condition_groups_collection_ptr_;
};

}  // namespace openscenario_visualization

#endif  // OPENSCENARIO_VISUALIZATION_PARAMETERS_HPP_
