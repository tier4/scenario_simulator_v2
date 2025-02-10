// Copyright 2022 TIER IV, Inc. All rights reserved.
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

class TrafficLightRunningScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit TrafficLightRunningScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "traffic_light_running",
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map", "lanelet2_map.osm",
      __FILE__, false, option)
  {
    start();
  }

private:
  const traffic_simulator::TrafficLight::Bulb green_{
    traffic_simulator::TrafficLight::Color::green,
    traffic_simulator::TrafficLight::Status::solid_on,
    traffic_simulator::TrafficLight::Shape::circle};
  const traffic_simulator::TrafficLight::Bulb red_{
    traffic_simulator::TrafficLight::Color::red, traffic_simulator::TrafficLight::Status::solid_on,
    traffic_simulator::TrafficLight::Shape::circle};
  void onUpdate() override
  {
    const auto t = api_.getCurrentTime();

    // traffic light control
    if (1.0 <= t && t <= 1.1) {
      api_.getConventionalTrafficLight(34836).emplace(red_);
      api_.getConventionalTrafficLight(34802).emplace(red_);
    }
    if (10.0 <= t && t <= 10.1) {
      api_.getConventionalTrafficLight(34836).clear();
      api_.getConventionalTrafficLight(34802).clear();
      api_.getConventionalTrafficLight(34836).emplace(green_);
      api_.getConventionalTrafficLight(34802).emplace(green_);
    }

    if (
      t < 10.1 &&
      static_cast<traffic_simulator::LaneletPose>(api_.getEntityStatus("ego").getLaneletPose())
          .lanelet_id == 34624) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (
      t >= 10.1 &&
      static_cast<traffic_simulator::LaneletPose>(api_.getEntityStatus("ego").getLaneletPose())
          .lanelet_id == 34624) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
    // LCOV_EXCL_STOP
    if (t >= 30) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
  }

  void onInitialize() override
  {
    //Vehicle setting
    api_.spawn(
      "ego", traffic_simulator::helper::constructCanonicalizedLaneletPose(34408, 0, 0),
      getVehicleParameters(), traffic_simulator::VehicleBehavior::contextGamma());
    api_.setEntityStatus(
      "ego", traffic_simulator::helper::constructCanonicalizedLaneletPose(34408, 0, 0),
      traffic_simulator::helper::constructActionStatus(10));
    api_.requestSpeedChange("ego", 5, true);
    api_.requestAssignRoute(
      "ego", std::vector<traffic_simulator::CanonicalizedLaneletPose>{
               traffic_simulator::helper::constructCanonicalizedLaneletPose(34630, 0.0, 0),
               traffic_simulator::helper::constructCanonicalizedLaneletPose(34696, 0.0, 0)});
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<TrafficLightRunningScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
