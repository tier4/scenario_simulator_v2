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

#ifndef CONCEALER__GET_PARAMETER_HPP_
#define CONCEALER__GET_PARAMETER_HPP_

#include <rclcpp/rclcpp.hpp>

namespace concealer
{
static constexpr auto default_architecture_type = "awf/universe/20240605";

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
  auto node = rclcpp::Node("get_parameter", "simulation");
  return getParameter(node.get_node_parameters_interface(), name, value);
}
}  // namespace concealer

#endif  // CONCEALER__GET_PARAMETER_HPP_
