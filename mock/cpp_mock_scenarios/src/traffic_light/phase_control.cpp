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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cpp_mock_scenarios/catalogs.hpp>
#include <cpp_mock_scenarios/cpp_scenario_node.hpp>
#include <openscenario_msgs/msg/driver_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/api/api.hpp>

// headers in STL
#include <memory>
#include <string>
#include <vector>

class AccelerateAndFollowScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit AccelerateAndFollowScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "idiot_npc", ament_index_cpp::get_package_share_directory("cargo_delivery") + "/maps/kashiwa",
      "lanelet2_map_with_private_road_and_walkway_ele_fix.osm", __FILE__, false, option)
  {
    start();
  }

private:
  void onUpdate() override
  {
    const auto t = api_.getCurrentTime();
    const auto color = api_.getTrafficLightColor(34802);
    if (0 < t && t <= 9.9) {
      if (color != traffic_simulator::TrafficLightColor::GREEN) {
        stop(cpp_mock_scenarios::Result::FAILURE);
      }
    }
    if (t >= 10.1) {
      if (color == traffic_simulator::TrafficLightColor::YELLOW) {
        stop(cpp_mock_scenarios::Result::SUCCESS);
      } else {
        stop(cpp_mock_scenarios::Result::FAILURE);
      }
    }
  }

  void onInitialize() override
  {
    api_.spawn(false, "ego", getVehicleParameters());
    api_.setEntityStatus(
      "ego", traffic_simulator::helper::constructLaneletPose(34741, 0, 0),
      traffic_simulator::helper::constructActionStatus(3));
    api_.setTargetSpeed("ego", 3, true);
    std::vector<std::pair<double, traffic_simulator::TrafficLightColor>> phase;
    phase = {
      {10, traffic_simulator::TrafficLightColor::GREEN},
      {10, traffic_simulator::TrafficLightColor::YELLOW},
      {10, traffic_simulator::TrafficLightColor::RED}};
    api_.setTrafficLightColorPhase(34802, phase);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<AccelerateAndFollowScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}