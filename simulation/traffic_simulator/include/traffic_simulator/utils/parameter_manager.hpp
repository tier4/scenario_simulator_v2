// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__UTILS__PARAMETER_MANAGER_HPP_
#define TRAFFIC_SIMULATOR__UTILS__PARAMETER_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>

namespace traffic_simulator
{

class ParameterManager
{
public:
  template <typename NodeT>
  ParameterManager(NodeT && node)
  : node_parameters_interface_(rclcpp::node_interfaces::get_node_parameters_interface(node))
  {
  }

  template <typename ParameterT>
  auto getParameter(const std::string & name, const ParameterT & default_value = {}) const
  {
    if (not node_parameters_interface_->has_parameter(name)) {
      node_parameters_interface_->declare_parameter(name, rclcpp::ParameterValue(default_value));
    }
    return node_parameters_interface_->get_parameter(name).get_value<ParameterT>();
  }

private:
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface_;
};

}  // namespace traffic_simulator

#endif  //TRAFFIC_SIMULATOR__UTILS__PARAMETER_MANAGER_HPP_
