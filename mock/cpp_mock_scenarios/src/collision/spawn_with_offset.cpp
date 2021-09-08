// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <quaternion_operation/quaternion_operation.h>

#include <cpp_mock_scenarios/catalogs.hpp>
#include <cpp_mock_scenarios/cpp_scenario_node.hpp>
#include <openscenario_msgs/msg/driver_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/api/api.hpp>

// headers in STL
#include <memory>
#include <string>
#include <vector>

class SpawnWithOffsetScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit SpawnWithOffsetScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode("spawn_with_offset", __FILE__, false, option)
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
    api_.spawn(false, "ego", getVehicleParameters());
    api_.setEntityStatus(
      "ego", traffic_simulator::helper::constructLaneletPose(34741, 0.2, 1.3),
      traffic_simulator::helper::constructActionStatus(0));
    api_.setTargetSpeed("ego", 0, true);
    api_.spawn(
      false, "bob", getPedestrianParameters(),
      traffic_simulator::helper::constructLaneletPose(34741, 0, -0.874),
      traffic_simulator::helper::constructActionStatus(0));
    api_.setTargetSpeed("bob", 0, true);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<SpawnWithOffsetScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}