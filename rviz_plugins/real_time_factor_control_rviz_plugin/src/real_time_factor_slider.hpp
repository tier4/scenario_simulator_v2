//
// Copyright 2023 Tier IV, Inc. All rights reserved.
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

#ifndef REAL_TIME_FACTOR_SLIDER_PANEL_HPP_
#define REAL_TIME_FACTOR_SLIDER_PANEL_HPP_

#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QSlider>

#ifndef Q_MOC_RUN

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#endif

#include <std_msgs/msg/float64.hpp>

namespace real_time_factor_control_rviz_plugin
{
class RealTimeFactorSliderPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit RealTimeFactorSliderPanel(QWidget * parent = nullptr);
  auto onInitialize() -> void override;

public Q_SLOTS:
  void onChangedRealTimeFactorValue(int real_time_factor_value);

protected:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr update_real_time_factor_publisher;

  QLabel * value_label_;
  QSlider * slider_;
};

}  // end namespace real_time_factor_control_rviz_plugin

#endif  // REAL_TIME_FACTOR_SLIDER_PANEL_HPP_
