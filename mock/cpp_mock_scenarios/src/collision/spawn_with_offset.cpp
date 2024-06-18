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
class SpawnWithOffsetScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit SpawnWithOffsetScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "spawn_with_offset", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "private_road_and_walkway_ele_fix/lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  void onUpdate() override
  {
    const auto t = api_.getCurrentTime();
    if (t > 5) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    } else {
      if (api_.checkCollision("ego", "bob")) {
      }
    }
  }

  void onInitialize() override
  {
    api_.spawn(
      "ego", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34741, 0.2, 1.3)),
      getVehicleParameters());
    api_.setLinearVelocity("ego", 0);
    api_.requestSpeedChange("ego", 0, true);

    api_.spawn(
      "bob", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34741, 0, -0.874)),
      getPedestrianParameters());
    api_.setLinearVelocity("bob", 0);
    api_.requestSpeedChange("bob", 0, true);
  }
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<cpp_mock_scenarios::SpawnWithOffsetScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
