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

#include <class_loader/class_loader.hpp>
#include <openscenario_visualization/context_panel_plugin.hpp>

#include "ui_context_panel_plugin.h"

namespace openscenario_visualization
{
ContextPanel::ContextPanel(QWidget * parent) : Panel(parent), ui_(new Ui::ContextPanel())
{
  node_ = std::make_shared<rclcpp::Node>("context_panel", "openscenario_visualization");
  topic_query_thread_ = std::thread(&ContextPanel::updateTopicCandidates, this);
  using namespace std::chrono_literals;
  ui_->setupUi(this);
  ui_->TopicSelect->addItem("--- Select Topics ---");
  connect(ui_->TopicSelect, SIGNAL(highlighted(int)), this, SLOT(selectTopic(int)));
}

ContextPanel::~ContextPanel() = default;

void ContextPanel::onInitialize() { parentWidget()->setVisible(true); }

void ContextPanel::onEnable()
{
  show();
  parentWidget()->show();
}

void ContextPanel::onDisable()
{
  hide();
  parentWidget()->hide();
}

void ContextPanel::updateTopicCandidates()
{
  while (rclcpp::ok()) {
    const auto topic_dict = node_->get_topic_names_and_types();
    topics_.clear();
    topic_candidates_mutex_.lock();
    for (const auto & data : topic_dict) {
      // std::cout << "topic : " << data.first << std::endl;
      const auto types = data.second;
      for (const auto & type : types) {
        if (type == "openscenario_interpreter_msgs/msg/Context") {
          topics_.emplace_back(data.first);
        }
      }
    }
    std::unique(topics_.begin(), topics_.end());
    topic_candidates_mutex_.unlock();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void ContextPanel::selectTopic(int)
{
  topic_candidates_mutex_.lock();
  ui_->TopicSelect->clear();
  ui_->TopicSelect->addItem("--- Select Topics ---");
  for (const auto & topic : topics_) {
    ui_->TopicSelect->addItem(topic.c_str());
  }
  topic_candidates_mutex_.unlock();
}
}  // namespace openscenario_visualization

CLASS_LOADER_REGISTER_CLASS(openscenario_visualization::ContextPanel, rviz_common::Panel)
