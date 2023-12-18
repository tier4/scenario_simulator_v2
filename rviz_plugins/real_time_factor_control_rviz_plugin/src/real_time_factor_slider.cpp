//
// Copyright 2020 Tier IV, Inc. All rights reserved.
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
//

#include "real_time_factor_slider.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <qt5/QtWidgets/QHBoxLayout>
#include <rviz_common/display_context.hpp>

namespace real_time_factor_control_rviz_plugin
{
RealTimeFactorSliderPanel::RealTimeFactorSliderPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  value_label_ = new QLabel("x 1.00");
  value_label_->setAlignment(Qt::AlignCenter);

  slider_ = new QSlider(Qt::Horizontal);
  slider_->setMinimum(1);
  slider_->setMaximum(200);
  slider_->setTickInterval(1);
  slider_->setValue(100);

  auto * layout = new QHBoxLayout();
  layout->addWidget(value_label_);
  layout->addWidget(slider_);

  setLayout(layout);
}

void RealTimeFactorSliderPanel::onChangedRealTimeFactorValue(int real_time_factor_value)
{
  std_msgs::msg::Float64 msg;
  msg.data = real_time_factor_value / 100.0;
  update_real_time_factor_publisher->publish(msg);
  value_label_->setText(QString("x ") + QString::number(msg.data, 'f', 2));
}

void RealTimeFactorSliderPanel::onInitialize()
{
  rclcpp::Node::SharedPtr raw_node =
    this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  update_real_time_factor_publisher =
    raw_node->create_publisher<std_msgs::msg::Float64>("/real_time_factor", 1);

  connect(slider_, SIGNAL(valueChanged(int)), SLOT(onChangedRealTimeFactorValue(int)));
}

}  // namespace real_time_factor_control_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
  real_time_factor_control_rviz_plugin::RealTimeFactorSliderPanel, rviz_common::Panel)
