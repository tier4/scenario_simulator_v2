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
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/api/api.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>

// headers in STL
#include <memory>
#include <string>
#include <vector>

namespace cpp_mock_scenarios
{
class SynchronizedActionWithSpeed : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit SynchronizedActionWithSpeed(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "synchronized_action_with_speed",
      ament_index_cpp::get_package_share_directory("simple_cross_map") + "/map", "lanelet2_map.osm",
      __FILE__, false, option)
  {
    start();
  }

private:
  bool requested = false;

  void onUpdate() override
  {
    auto npc = api_.getEntity("npc");
    auto ego = api_.getEntity("ego");
    static const auto ego_target = traffic_simulator::helper::constructLaneletPose(147, 0, 0);
    static const auto npc_target = traffic_simulator::helper::constructLaneletPose(133, 0, 0);

    // SUCCESS
    if (
      npc->requestSynchronize("ego", ego_target, npc_target, 2, 0.5) &&
      ego->isInPosition(ego_target, 1.0) && npc->isInPosition(npc_target, 1.0) &&
      npc->getCurrentTwist().linear.x < 2.5) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }

    // FAILURES
    if (api_.getCurrentTime() >= 9.0) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (api_.checkCollision("ego", "npc")) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
  }

  void onInitialize() override
  {
    auto ego = api_.spawn(
      "ego", traffic_simulator::helper::constructLaneletPose(7, 40, 0), getVehicleParameters());
    ego->setLinearVelocity(3);
    ego->requestSpeedChange(3, true);
    ego->requestAssignRoute({traffic_simulator::helper::constructLaneletPose(154, 20, 0)});

    auto npc = api_.spawn(
      "npc", traffic_simulator::helper::constructLaneletPose(14, 15, 0), getVehicleParameters());
    npc->requestAssignRoute({traffic_simulator::helper::constructLaneletPose(140, 20, 0)});
    npc->setLinearVelocity(6);
  }
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<cpp_mock_scenarios::SynchronizedActionWithSpeed>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
