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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cpp_mock_scenarios/catalogs.hpp>
#include <cpp_mock_scenarios/cpp_scenario_node.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/api/api.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <vector>

namespace cpp_mock_scenarios
{
/// @note Test case to verify if the do_nothing plugin can be loaded.
class LoadDoNothingPluginScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit LoadDoNothingPluginScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "crashing_npc", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "private_road_and_walkway_ele_fix/lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  void onUpdate() override
  {
    /// @note When using the do_nothing plugin, the return value of the `getCurrentAction` function is always do_nothing.
    if (
      api_.getCurrentAction("ego") != "do_nothing" ||
      api_.getCurrentAction("pedestrian") != "do_nothing") {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (
      api_.getCurrentAction("vehicle_spawn_with_behavior_tree") == "do_nothing" ||
      api_.getCurrentAction("pedestrian_spawn_with_behavior_tree") == "do_nothing") {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    api_.resetBehaviorPlugin(
      "vehicle_spawn_with_behavior_tree",
      traffic_simulator::entity::VehicleEntity::BuiltinBehavior::doNothing());
    api_.resetBehaviorPlugin(
      "pedestrian_spawn_with_behavior_tree",
      traffic_simulator::entity::PedestrianEntity::BuiltinBehavior::doNothing());
    if (
      api_.getCurrentAction("vehicle_spawn_with_behavior_tree") != "do_nothing" ||
      api_.getCurrentAction("pedestrian_spawn_with_behavior_tree") != "do_nothing") {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }

    stop(cpp_mock_scenarios::Result::SUCCESS);
  }
  void onInitialize() override
  {
    api_.spawn(
      "ego", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34741, 0, 0)),
      getVehicleParameters(),
      traffic_simulator::entity::VehicleEntity::BuiltinBehavior::doNothing());
    api_.spawn(
      "pedestrian", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34741, 3, 0)),
      getPedestrianParameters(),
      traffic_simulator::entity::PedestrianEntity::BuiltinBehavior::doNothing());
    api_.spawn(
      "vehicle_spawn_with_behavior_tree",
      api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34741, 2.0, 0)),
      getVehicleParameters(),
      traffic_simulator::entity::VehicleEntity::BuiltinBehavior::behaviorTree());
    api_.spawn(
      "pedestrian_spawn_with_behavior_tree",
      api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34741, 3, 0)),
      getPedestrianParameters(),
      traffic_simulator::entity::PedestrianEntity::BuiltinBehavior::behaviorTree());
  }
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<cpp_mock_scenarios::LoadDoNothingPluginScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
