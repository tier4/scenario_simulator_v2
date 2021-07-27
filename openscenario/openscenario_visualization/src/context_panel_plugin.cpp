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
 
#include <openscenario_visualization/context_panel_plugin.hpp>
#include <class_loader/class_loader.hpp>

#include "ui_context_panel.h"

namespace openscenario_visualization
{
ContextPanel::ContextPanel(QWidget* parent) : Panel(parent)
{
  node_ = std::make_shared<rclcpp::Node>("context_panel", "openscenario_visualization");
  ui_->setupUi(this);
}

ContextPanel::~ContextPanel() = default;

void ContextPanel::onInitialize()
{
  parentWidget()->setVisible(true);
}

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
}  // namespace openscenario_visualization

CLASS_LOADER_REGISTER_CLASS(openscenario_visualization::ContextPanel, rviz_common::Panel)
