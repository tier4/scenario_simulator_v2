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
class SpawnInMapFrameScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit SpawnInMapFrameScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "stop_at_crosswalk", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  void onUpdate() override
  {
    const auto map_pose = traffic_simulator::pose::toMapPose(
      traffic_simulator::helper::constructCanonicalizedLaneletPose(
        120545, 0.0, 0.0, api_.getHdmapUtils()));
    if (api_.reachPosition("ego", map_pose, 0.1)) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    } else {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
  }

  void onInitialize() override
  {
    api_.spawn(
      "ego",
      traffic_simulator::pose::toMapPose(
        traffic_simulator::helper::constructCanonicalizedLaneletPose(
          120545, 0.0, 0.0, api_.getHdmapUtils())),
      getVehicleParameters());
  }
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<cpp_mock_scenarios::SpawnInMapFrameScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
