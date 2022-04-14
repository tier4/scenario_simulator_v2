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

#ifndef OPENSCENARIO_VISUALIZATION__CONTEXT_PANEL_PLUGIN_HPP_
#define OPENSCENARIO_VISUALIZATION__CONTEXT_PANEL_PLUGIN_HPP_

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif

#include <mutex>
#include <openscenario_interpreter_msgs/msg/context.hpp>
#include <openscenario_visualization/context_panel_plugin.hpp>
#include <rviz_common/panel.hpp>
#include <string>
#include <thread>
#include <vector>

namespace Ui
{
class ContextPanel;
}

namespace openscenario_visualization
{
class ContextPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  ContextPanel(QWidget * parent = nullptr);
  ~ContextPanel() override;

  void onInitialize() override;
  void onEnable();
  void onDisable();
private Q_SLOTS:
  void selectTopic(int);
  void update_display();
Q_SIGNALS:
  void display_trigger();

protected:
  Ui::ContextPanel * ui_;
  rclcpp::Node::SharedPtr node_;
  std::vector<std::string> topics_;

private:
  std::thread topic_query_thread_;
  std::thread spin_thread_;
  void updateTopicCandidates();
  std::mutex topic_candidates_mutex_;
  bool selected_ = false;
  rclcpp::Subscription<openscenario_interpreter_msgs::msg::Context>::SharedPtr context_sub_;
  void startSubscription();
  void contextCallback(const openscenario_interpreter_msgs::msg::Context::SharedPtr msg);
  void spin();
  std::string context_;
  double simulation_time_;
  std::vector<std::string> item_vec_;
  std::vector<std::vector<std::string>> condition_group_vec_;
  bool runnning_ = true;
};
}  // namespace openscenario_visualization

#endif  // OPENSCENARIO_VISUALIZATION__CONTEXT_PANEL_PLUGIN_HPP_
