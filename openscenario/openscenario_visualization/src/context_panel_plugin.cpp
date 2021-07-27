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
  using namespace std::chrono_literals;
  ui_->setupUi(this);
  ui_->TopicSelect->addItem("/simulation/context");
  topic_query_thread_ = std::thread(&ContextPanel::updateTopicCandidates, this);
  connect(ui_->TopicSelect, SIGNAL(highlighted(int)), this, SLOT(selectTopic(int)));
  startSubscription();
  spin_thread_ = std::thread(&ContextPanel::spin, this);
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

void ContextPanel::contextCallback(const openscenario_interpreter_msgs::msg::Context::SharedPtr msg)
{
  context_ = msg->data;
  std::cout << "context : " << context_ << std::endl;
}

void ContextPanel::updateTopicCandidates()
{
  while (rclcpp::ok()) {
    if (selected_) {
      break;
    }
    const auto topic_dict = node_->get_topic_names_and_types();
    topics_.clear();
    topic_candidates_mutex_.lock();
    for (const auto & data : topic_dict) {
      const auto types = data.second;
      for (const auto & type : types) {
        if (type == "openscenario_interpreter_msgs/msg/Context") {
          topics_.emplace_back(data.first);
        }
      }
    }
    std::unique(topics_.begin(), topics_.end());
    ui_->TopicSelect->clear();
    ui_->TopicSelect->addItem("simulation/context");
    for (const auto & topic : topics_) {
      if (topic != "/simulation/context") {
        ui_->TopicSelect->addItem(topic.c_str());
      }
    }
    topic_candidates_mutex_.unlock();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void ContextPanel::spin()
{
  while (rclcpp::ok()) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void ContextPanel::startSubscription()
{
  std::string topic = ui_->TopicSelect->currentText().toStdString();
  context_sub_ = node_->create_subscription<openscenario_interpreter_msgs::msg::Context>(
    topic, 1, std::bind(&ContextPanel::contextCallback, this, std::placeholders::_1));
}

void ContextPanel::selectTopic(int)
{
  topic_candidates_mutex_.lock();
  selected_ = true;
  topic_candidates_mutex_.unlock();
}
}  // namespace openscenario_visualization

CLASS_LOADER_REGISTER_CLASS(openscenario_visualization::ContextPanel, rviz_common::Panel)
