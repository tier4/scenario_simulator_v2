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

#ifndef TRAFFIC_SIMULATOR__UTILS__NODE_PARAMETER_HANDLER_HPP_
#define TRAFFIC_SIMULATOR__UTILS__NODE_PARAMETER_HANDLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>

namespace traffic_simulator
{
class NodeParameterHandler
{
public:
  template <typename NodeT>
  explicit NodeParameterHandler(NodeT && node)
  : node_parameters_interface_(
      rclcpp::node_interfaces::get_node_parameters_interface(std::forward<NodeT>(node)))
  {
  }

  /**
   * Get parameter or declare it if it has not been declared before. Declare it with a default value.
   * @param name The name of the parameter
   * @param default_value The default value of the parameter
   * @return The value of the parameter
   */
  template <typename ParameterT>
  auto getParameter(const std::string & name, const ParameterT & default_value = {}) const
    -> ParameterT
  {
    if (not node_parameters_interface_->has_parameter(name)) {
      node_parameters_interface_->declare_parameter(name, rclcpp::ParameterValue(default_value));
    }
    return node_parameters_interface_->get_parameter(name).get_value<ParameterT>();
  }

private:
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface_;
};
}  // namespace traffic_simulator

#endif  //TRAFFIC_SIMULATOR__UTILS__NODE_PARAMETER_HANDLER_HPP_
