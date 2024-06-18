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
class CrashingNpcScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit CrashingNpcScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "crashing_npc", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "private_road_and_walkway_ele_fix/lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  void onUpdate() override
  {
    double current_time = api_.getCurrentTime();
    if (api_.checkCollision("ego", "npc")) {
      // LCOV_EXCL_START
      if (current_time <= 3.0) {
        stop(cpp_mock_scenarios::Result::FAILURE);
        // LCOV_EXCL_STOP
      } else {
        stop(cpp_mock_scenarios::Result::SUCCESS);
      }
    }
  }
  void onInitialize() override
  {
    api_.spawn(
      "ego", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34741, 0, 0)),
      getVehicleParameters());
    api_.setLinearVelocity("ego", 0.0);
    api_.requestSpeedChange("ego", 15, true);
    traffic_simulator_msgs::msg::BehaviorParameter behavior_parameter;
    behavior_parameter.see_around = false;
    api_.setBehaviorParameter("ego", behavior_parameter);

    api_.spawn(
      "npc", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34741, 10, 0)),
      getVehicleParameters());
    api_.setLinearVelocity("npc", 0.0);
    api_.requestSpeedChange("npc", 5, true);
  }
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<cpp_mock_scenarios::CrashingNpcScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
