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

#include "openscenario_visualization_plugins/openscenario_visualization_condition_groups_plugin.hpp"

#include <OgreHardwarePixelBuffer.h>
#include <X11/Xlib.h>

#include <QPainter>
#include <algorithm>
#include <iomanip>
#include <rviz_common/display_context.hpp>
#include <rviz_common/uniform_string_stream.hpp>
#include <string>
#include <vector>

namespace openscenario_visualization
{
VisualizationConditionGroupsDisplay::VisualizationConditionGroupsDisplay()
: condition_groups_collection_ptr_(std::make_shared<std::vector<ConditionGroups>>())
{
  /// @note Get screen info of default display
  const Screen * screen_info = DefaultScreenOfDisplay(XOpenDisplay(NULL));

  /// @note Fixed height for a 4k resolution screen
  /// @sa https://github.com/tier4/scenario_simulator_v2/pull/1033#discussion_r1412781103
  constexpr float hight_4k = 2160.0;

  /// @note Calculate scale based on current screen height and 4k height
  const float scale = static_cast<float>(screen_info->height) / hight_4k;

  /**
   * @note The multiplication of 35.0 is used to determine the initial text size in the panel. 
   * This value is a base size that is then scaled according to the screen resolution.
   * The 'Value Scale' property allows users to adjust the scaling factor of this text size later on, 
   * but the initial value of 35.0 is set to ensure a default size that is likely suitable for most screens. 
   * The scaling factor adjusts this size to ensure readability across various resolutions.
   */
  const float text_size = scale * 35.0;

  /// @note Define initial value of left edge position of condition results panel
  const int left = 0;

  /**
   * @note Define initial value of top edge position of condition results panel
   * The multiplication of 450 sets the initial top edge position of the condition results panel. 
   * Like the width and length, this is a predefined base value, not necessarily linked to any value set from the Top property.
   * The purpose of this calculation is to position the top edge of the panel at an appropriate place on the screen, 
   * again scaling according to screen resolution to maintain a consistent look across different devices.
   */
  const int top = static_cast<int>(std::round(450 * scale));

  /**
   * @note Define initial value of horizontal length of condition results panel.
   * The reason 2000 is hard-coded here is because that number displayed most beautifully when we tested the operation on a 4K/non 4K display.
   * Also, this number can be set via the rviz GUI.
   */
  const int length = static_cast<int>(std::round(2000 * scale));

  /**
   * @note Define initial value of width of condition results panel.
   * The reason 2000 is hard-coded here is because that number displayed most beautifully when we tested the operation on a 4K/non 4K display.
   * Also, this number can be set via the rviz GUI.
   */
  const int width = static_cast<int>(std::round(2000 * scale));

  property_topic_name_ = new rviz_common::properties::StringProperty(
    "Topic", "/simulation/context", "The topic on which to publish simulation context.", this,
    SLOT(updateTopic()), this);
  property_text_color_ = new rviz_common::properties::ColorProperty(
    "Text Color", QColor(255, 255, 255), "text color", this, SLOT(updateVisualization()), this);
  property_left_ = new rviz_common::properties::IntProperty(
    "Left", left, "Left of the plotter window", this, SLOT(updateVisualization()), this);
  property_left_->setMin(0);
  property_top_ = new rviz_common::properties::IntProperty(
    "Top", top, "Top of the plotter window", this, SLOT(updateVisualization()));
  property_top_->setMin(0);

  property_length_ = new rviz_common::properties::IntProperty(
    "Length", length, "Length of the plotter window", this, SLOT(updateVisualization()), this);
  property_length_->setMin(10);
  property_width_ = new rviz_common::properties::IntProperty(
    "Width", width, "Width of the plotter window", this, SLOT(updateVisualization()), this);
  property_width_->setMin(10);
  property_value_scale_ = new rviz_common::properties::FloatProperty(
    "Value Scale", text_size,
    "This property controls the scaling factor for the text size on the panel. Setting a higher "
    "value results in larger text, making the displayed information easier to read.",
    this, SLOT(updateVisualization()), this);
  property_value_scale_->setMin(0.01);
}

VisualizationConditionGroupsDisplay::~VisualizationConditionGroupsDisplay()
{
  if (initialized()) {
    overlay_->hide();
  }
}

void VisualizationConditionGroupsDisplay::onInitialize()
{
  static int count = 0;
  rviz_common::UniformStringStream ss;
  ss << "VisualizationConditionGroupsDisplayObject" << count++;

  overlay_.reset(new jsk_rviz_plugins::OverlayObject(
    scene_manager_, context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger(),
    ss.str()));
  overlay_->show();

  overlay_->updateTextureSize(property_width_->getInt(), property_length_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
  updateVisualization();
}

void VisualizationConditionGroupsDisplay::updateTopic()
{
  unsubscribe();
  subscribe();
}

void VisualizationConditionGroupsDisplay::onEnable()
{
  subscribe();
  overlay_->show();
}

void VisualizationConditionGroupsDisplay::onDisable()
{
  unsubscribe();
  reset();
  overlay_->hide();
}

void VisualizationConditionGroupsDisplay::subscribe()
{
  std::string topic_name = property_topic_name_->getStdString();
  if (topic_name.length() > 0 && topic_name != "/") {
    rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    simulation_context_sub_ = raw_node->create_subscription<Context>(
      topic_name, rclcpp::QoS{1}.transient_local(),
      std::bind(&VisualizationConditionGroupsDisplay::processMessage, this, std::placeholders::_1));
  }
}

void VisualizationConditionGroupsDisplay::unsubscribe() { simulation_context_sub_.reset(); }

void VisualizationConditionGroupsDisplay::processMessage(const Context::ConstSharedPtr msg_ptr)
{
  if (!overlay_->isVisible()) return;

  // Create a QImage and fill it with transparent color.
  QColor background_color;
  background_color.setAlpha(0);
  jsk_rviz_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage hud = buffer.getQImage(*overlay_);
  hud.fill(background_color);

  QPainter painter(&hud);
  painter.setRenderHint(QPainter::Antialiasing, true);
  // QColor text_color = property_text_color_->getColor();
  QColor text_color(property_text_color_->getColor());
  text_color.setAlpha(255);
  painter.setPen(QPen(text_color, static_cast<int>(2), Qt::SolidLine));
  QFont font = painter.font();
  font.setPixelSize(std::max(static_cast<int>(property_value_scale_->getFloat()), 1));
  font.setBold(true);
  painter.setFont(font);

  loadConditionGroups(msg_ptr);

  std::ostringstream context_ss;
  for (const auto & condition_groups : *condition_groups_collection_ptr_) {
    context_ss << std::fixed << std::setprecision(0)
               << "Condition Groups Name: " << condition_groups.groups_name << std::endl;
    for (const auto & condition_group : condition_groups.condition_groups) {
      for (const auto & condition : condition_group.conditions) {
        context_ss << "    Current Evaluation: " << condition.current_evaluation << std::endl
                   << "    Current Value: " << condition.current_value << std::endl
                   << "    Type: " << condition.type << std::endl;
      }
    }
    context_ss << std::endl;
  }

  painter.drawText(
    property_left_->getInt(), property_top_->getInt(), overlay_->getTextureWidth(),
    overlay_->getTextureHeight(), Qt::AlignLeft | Qt::AlignTop, context_ss.str().c_str());

  painter.end();
  last_msg_ptr_ = msg_ptr;
}

void VisualizationConditionGroupsDisplay::updateVisualization()
{
  int width = property_width_->getInt();
  int height = property_length_->getInt();

  overlay_->updateTextureSize(width, height);
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(width, height);

  if (last_msg_ptr_) {
    processMessage(last_msg_ptr_);
  }
}

void VisualizationConditionGroupsDisplay::loadConditionGroups(const Context::ConstSharedPtr msg_ptr)
{
  if (!msg_ptr) return;

  YAML::Node data;
  try {
    data = YAML::Load(msg_ptr->data);
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("Failed to load YAML: ") + e.what());
  }

  condition_groups_collection_ptr_->clear();

  auto stories = data["OpenSCENARIO"]["Storyboard"]["Story"];
  for (const auto & story : stories) {
    processStory(story);
  }
}

void VisualizationConditionGroupsDisplay::processStory(const YAML::Node & story_node)
{
  for (const auto & act : story_node["Act"]) {
    for (const auto & maneuver_group : act["ManeuverGroup"]) {
      for (const auto & maneuver : maneuver_group["Maneuver"]) {
        processManeuver(maneuver);
      }
    }
  }
}

void VisualizationConditionGroupsDisplay::processManeuver(const YAML::Node & maneuver_node)
{
  for (const auto & event : maneuver_node["Event"]) {
    if (event["StartTrigger"] && event["StartTrigger"]["ConditionGroup"]) {
      processEvent(event);
    }
  }
}

void VisualizationConditionGroupsDisplay::processEvent(const YAML::Node & event_node)
{
  std::string event_name;
  try {
    event_name = event_node["name"].as<std::string>();
  } catch (const YAML::BadConversion & e) {
    event_name = "";
  }
  if (event_name.empty()) {
    event_name = "name is not defined";
  }

  ConditionGroups condition_groups;
  condition_groups.groups_name = event_name;

  for (const auto & condition_group_node : event_node["StartTrigger"]["ConditionGroup"]) {
    ConditionGroup condition_group;

    for (const auto & condition_node : condition_group_node["Condition"]) {
      Condition condition_msg;
      condition_msg.current_evaluation = condition_node["currentEvaluation"].as<std::string>();
      condition_msg.current_value = condition_node["currentValue"].as<std::string>();
      condition_msg.type = condition_node["type"].as<std::string>();

      condition_group.conditions.push_back(condition_msg);
    }

    condition_groups.condition_groups.push_back(condition_group);
  }

  condition_groups_collection_ptr_->push_back(condition_groups);
}

}  // namespace openscenario_visualization

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  openscenario_visualization::VisualizationConditionGroupsDisplay, rviz_common::Display)
