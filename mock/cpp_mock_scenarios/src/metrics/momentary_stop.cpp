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
      "idiot_npc", ament_index_cpp::get_package_share_directory("cargo_delivery") + "/maps/kashiwa",
      "lanelet2_map_with_private_road_and_walkway_ele_fix.osm", __FILE__, false, option)
  {
    start();
  }

private:
  void onUpdate() override
  {
    if (
      api_.metricExists("ego_momentary_stop") &&
      api_.getMetricLifecycle("ego_momentary_stop") == metrics::MetricLifecycle::FAILURE) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (
      api_.metricExists("ego_momentary_stop") &&
      api_.getMetricLifecycle("ego_momentary_stop") == metrics::MetricLifecycle::SUCCESS) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
  }

  void onInitialize() override
  {
    api_.spawn(false, "ego", getVehicleParameters());
    api_.setEntityStatus(
      "ego", traffic_simulator::helper::constructLaneletPose(34606, 20.0),
      traffic_simulator::helper::constructActionStatus(10));
    api_.setTargetSpeed("ego", 10, true);
    api_.addMetric<metrics::MomentaryStopMetric>(
      "ego_momentary_stop", "ego", -10, 10, 120635,
      metrics::MomentaryStopMetric::StopTargetLaneletType::STOP_LINE, 30, 1, 0.05);
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
