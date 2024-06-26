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

#ifndef TRAFFIC_SIMULATOR__UTILS__NODE_PARAMETERS_HPP_
#define TRAFFIC_SIMULATOR__UTILS__NODE_PARAMETERS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>

namespace traffic_simulator
{
/**
 * Get parameter or declare it if it has not been declared before. Declare it with a default value.
 * @param node_parameters The node parameters interface pointer, it will be used to access the parameters
 * @param name The name of the parameter
 * @param default_value The default value of the parameter
 * @return The value of the parameter
 */
template <typename ParameterT>
auto getParameter(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
  const std::string & name, const ParameterT & default_value = {}) -> ParameterT
{
  if (not node_parameters->has_parameter(name)) {
    node_parameters->declare_parameter(name, rclcpp::ParameterValue(default_value));
  }
  return node_parameters->get_parameter(name).get_value<ParameterT>();
}
}  // namespace traffic_simulator

#endif  //TRAFFIC_SIMULATOR__UTILS__NODE_PARAMETERS_HPP_
