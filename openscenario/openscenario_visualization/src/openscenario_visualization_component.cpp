// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include <openscenario_visualization/openscenario_visualization_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace openscenario_visualization
{
OpenscenarioVisualizationComponent::OpenscenarioVisualizationComponent(
  const rclcpp::NodeOptions & options)
: LifecycleNode("openscenario_visualization", options)
{
}

using Result = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
Result OpenscenarioVisualizationComponent::on_configure(const rclcpp_lifecycle::State &)
{
}
Result OpenscenarioVisualizationComponent::on_activate(const rclcpp_lifecycle::State &)
{
}
Result OpenscenarioVisualizationComponent::on_deactivate(const rclcpp_lifecycle::State &)
{
}
Result OpenscenarioVisualizationComponent::on_cleanup(const rclcpp_lifecycle::State &)
{
}
Result OpenscenarioVisualizationComponent::on_shutdown(const rclcpp_lifecycle::State &)
{
}
Result OpenscenarioVisualizationComponent::on_error(const rclcpp_lifecycle::State &)
{
}
}  // namespace openscenario_visualization

RCLCPP_COMPONENTS_REGISTER_NODE(openscenario_visualization::OpenscenarioVisualizationComponent)
