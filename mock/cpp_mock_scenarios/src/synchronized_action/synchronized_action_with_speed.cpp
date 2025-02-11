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
class SynchronizedActionWithSpeed : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit SynchronizedActionWithSpeed(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "synchronized_action_with_speed",
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map", "lanelet2_map.osm",
      __FILE__, false, option)
  {
    start();
  }

private:
  const traffic_simulator::CanonicalizedLaneletPose ego_target =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(34585, 0, 0);
  const traffic_simulator::CanonicalizedLaneletPose npc_target =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(34570, 0, 0);

  void onUpdate() override
  {
    auto & npc = api_.getEntity("npc");
    auto & ego = api_.getEntity("ego");

    // SUCCESS
    if (
      npc.requestSynchronize("ego", ego_target, npc_target, 2, 0.5) &&
      ego.isNearbyPosition(ego_target, 1.0) && npc.isNearbyPosition(npc_target, 1.0) &&
      npc.getCurrentTwist().linear.x < 2.5) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }

    // FAILURES
    if (api_.getCurrentTime() >= 30.0) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (api_.checkCollision("ego", "npc")) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
  }
  void onInitialize() override
  {
    api_.spawn(
      "ego", traffic_simulator::helper::constructCanonicalizedLaneletPose(34976, 20, 0),
      getVehicleParameters());

    auto & ego = api_.getEntity("ego");
    ego.setLinearVelocity(3);
    ego.requestSpeedChange(3, true);

    std::vector<geometry_msgs::msg::Pose> goal_poses;
    goal_poses.emplace_back(
      traffic_simulator::helper::constructCanonicalizedLaneletPose(34579, 20, 0));
    ego.requestAssignRoute(goal_poses);

    api_.spawn(
      "npc", traffic_simulator::helper::constructCanonicalizedLaneletPose(34576, 0, 0),
      getVehicleParameters());

    auto & npc = api_.getEntity("npc");

    std::vector<geometry_msgs::msg::Pose> npc_goal_poses;
    npc_goal_poses.emplace_back(
      traffic_simulator::helper::constructCanonicalizedLaneletPose(34564, 20, 0));
    npc.requestAssignRoute(npc_goal_poses);
    npc.setLinearVelocity(6);
  }

  auto getSampleLaneletPose(const traffic_simulator::LaneletPose & lanelet_pose)
    -> std::optional<traffic_simulator::CanonicalizedLaneletPose>
  {
    return traffic_simulator::pose::toCanonicalizedLaneletPose(lanelet_pose);
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
