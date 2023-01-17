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

#include <quaternion_operation/quaternion_operation.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cpp_mock_scenarios/catalogs.hpp>
#include <cpp_mock_scenarios/cpp_scenario_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/api/api.hpp>

// headers in STL
#include <memory>
#include <string>
#include <vector>

class MomentaryStopScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit MomentaryStopScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "momentary_stop", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "private_road_and_walkway_ele_fix/lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  void onUpdate() override
  {
    if (api_.getCurrentTime() >= 12) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
  }

  void onInitialize() override
  {
    api_.spawn(
      "ego", traffic_simulator::helper::constructLaneletPose(34606, 20.0), getVehicleParameters());
    api_.setLinearVelocity("ego", 7);
    api_.requestSpeedChange("ego", 7, true);
    api_.monitorStopLineMomentaryStop("ego", -10, 10, 120635, 30, 1, 0.05);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<MomentaryStopScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
