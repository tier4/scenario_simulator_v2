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
#include <nlohmann/json.hpp>
#include <openscenario_visualization/context_panel_plugin.hpp>

#include "ui_context_panel_plugin.h"

using json = nlohmann::json;

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
  connect(this, SIGNAL(display_trigger()), this, SLOT(update_display()));
  startSubscription();
  spin_thread_ = std::thread(&ContextPanel::spin, this);
}

ContextPanel::~ContextPanel() { runnning_ = false; }

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
  simulation_time_ = msg->time;
  json j_ = json::parse(context_);
  condition_group_vec_.clear();
  item_vec_.clear();
  auto story_json = j_["OpenSCENARIO"]["Storyboard"]["Story"];
  for (json::iterator it1 = story_json.begin(); it1 != story_json.end(); ++it1) {
    for (json::iterator it2 = (*it1)["Act"].begin(); it2 != (*it1)["Act"].end(); ++it2) {
      for (json::iterator it3 = (*it2)["ManeuverGroup"].begin();
           it3 != (*it2)["ManeuverGroup"].end(); ++it3) {
        for (json::iterator it4 = (*it3)["Maneuver"].begin(); it4 != (*it3)["Maneuver"].end();
             ++it4) {
          for (json::iterator it5 = (*it4)["Event"].begin(); it5 != (*it4)["Event"].end(); ++it5) {
            for (json::iterator it6 = (*it5)["StartTrigger"]["ConditionGroup"].begin();
                 it6 != (*it5)["StartTrigger"]["ConditionGroup"].end(); ++it6) {
              for (json::iterator it7 = (*it6)["Condition"].begin();
                   it7 != (*it6)["Condition"].end(); ++it7) {
                auto condition_ = (*it7);
                item_vec_.push_back(condition_["currentEvaluation"].dump());
                item_vec_.push_back(condition_["currentValue"].dump());
                item_vec_.push_back(condition_["name"].dump());
                item_vec_.push_back(condition_["type"].dump());
                condition_group_vec_.push_back(item_vec_);
                item_vec_.clear();
              }
            }
          }
        }
      }
    }
  }
  std::sort(condition_group_vec_.begin(), condition_group_vec_.end(), [](auto & x, auto & y) {
    return x[3] < y[3];
  });
  condition_group_vec_.erase(
    std::unique(condition_group_vec_.begin(), condition_group_vec_.end()),
    condition_group_vec_.end());
  display_trigger();
}

void ContextPanel::updateTopicCandidates()
{
  while (runnning_) {
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

void ContextPanel::update_display()
{
  if (not condition_group_vec_.empty()) {
    std::vector<std::vector<std::vector<std::string>>::iterator> true_it, false_it;
    for (std::vector<std::vector<std::string>>::iterator it = condition_group_vec_.begin();
         it != condition_group_vec_.end(); ++it) {
      if ((*it)[1].size() == 6) {
        true_it.push_back(it);
      } else {
        false_it.push_back(it);
      }
    }

    if (true_it.size() == 0) {
      ui_->success_condition_name->clear();
      ui_->success_condition_evaluation->clear();
      ui_->success_condition_status->clear();
    } else if (false_it.size() == 0) {
      ui_->failure_condition_name->clear();
      ui_->failure_condition_evaluation->clear();
      ui_->failure_condition_status->clear();
    }

    for (std::vector<std::vector<std::vector<std::string>>::iterator>::iterator i =
           false_it.begin();
         i != false_it.end(); i++) {
      if (i == false_it.begin()) {
        ui_->failure_condition_name->setText(QString::fromUtf8((*(*i))[3].c_str()));
        ui_->failure_condition_evaluation->setText(QString::fromUtf8((*(*i))[0].c_str()));
        ui_->failure_condition_status->setText(QString::fromUtf8((*(*i))[1].c_str()));
      } else {
        ui_->failure_condition_name->append(QString::fromUtf8((*(*i))[3].c_str()));
        ui_->failure_condition_evaluation->append(QString::fromUtf8((*(*i))[0].c_str()));
        ui_->failure_condition_status->append(QString::fromUtf8((*(*i))[1].c_str()));
      }
      ui_->failure_condition_status->setTextColor(QColor("red"));
    }
    for (std::vector<std::vector<std::vector<std::string>>::iterator>::iterator i = true_it.begin();
         i != true_it.end(); i++) {
      if (i == true_it.begin()) {
        ui_->success_condition_name->setText(QString::fromUtf8((*(*i))[3].c_str()));
        ui_->success_condition_evaluation->setText(QString::fromUtf8((*(*i))[0].c_str()));
        ui_->success_condition_status->setText(QString::fromUtf8((*(*i))[1].c_str()));
      } else {
        ui_->success_condition_name->append(QString::fromUtf8((*(*i))[3].c_str()));
        ui_->success_condition_evaluation->append(QString::fromUtf8((*(*i))[0].c_str()));
        ui_->success_condition_status->append(QString::fromUtf8((*(*i))[1].c_str()));
      }
      ui_->success_condition_status->setTextColor(QColor("blue"));
    }
    ui_->simulation_time_data->setText(QString::number(simulation_time_));
  }
}

void ContextPanel::spin()
{
  while (runnning_) {
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
  update_display();
}
}  // namespace openscenario_visualization

CLASS_LOADER_REGISTER_CLASS(openscenario_visualization::ContextPanel, rviz_common::Panel)
