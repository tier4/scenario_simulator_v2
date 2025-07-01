// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#ifndef COMMON__GET_PARAMETER_HPP_
#define COMMON__GET_PARAMETER_HPP_

#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

namespace common
{
/**
 * @brief Get the Node object for parameter obtaining.
 * This provides a one node for parameter obtaining for the whole executable, as opposed to template
 * function that creates a new node for each type of parameter. To guarantee the node name is unique
 * (Autoware in some versions requires this to run) we append the process id.
 */
inline auto getParameterNode() -> rclcpp::Node &
{
  static rclcpp::Node node{
    [](std::string name_base) { return name_base + "_pid" + std::to_string(getpid()); }(__func__),
    "simulation",
    // NOTE: enable automatically_declare_parameters_from_overrides to read parameters from yaml without declaration
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)};
  return node;
}

template <typename T>
auto getParameter(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node,
  const std::string & name, T value = {})
{
  if (not node->has_parameter(name)) {
    node->declare_parameter(name, rclcpp::ParameterValue(value));
  }
  return node->get_parameter(name).get_value<T>();
}

template <typename T>
auto getParameter(const std::string & name, T value = {})
{
  return getParameter(getParameterNode().get_node_parameters_interface(), name, value);
}
}  // namespace common

#endif  // COMMON__GET_PARAMETER_HPP_
