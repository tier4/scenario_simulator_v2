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
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>

// headers in STL
#include <memory>
#include <string>
#include <vector>

namespace cpp_mock_scenarios
{
class SynchronizedAction : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit SynchronizedAction(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "synchronized_action",
      ament_index_cpp::get_package_share_directory("simple_cross_map") + "/map", "lanelet2_map.osm",
      __FILE__, false, option)
  {
    start();
  }

private:
  bool requested = false;
  const traffic_simulator::CanonicalizedLaneletPose ego_target =
    api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34585, 0, 0, 0, 0, 0));
  const traffic_simulator::CanonicalizedLaneletPose npc_target =
    api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34570, 0, 0, 0, 0, 0));

  void onUpdate() override
  {
    // SUCCESS
    if (
      api_.requestSynchronize("npc", "ego", ego_target, npc_target, 1.0, 1.0, 50) &&
      api_.reachPosition("ego", ego_target, 1.0) && api_.reachPosition("npc", npc_target, 1.0)) {
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
    api_.spawn(
      "ego",
      api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34976, 20, 0, 0, 0, 0)),
      getVehicleParameters());
    api_.setLinearVelocity("ego", 3);
    api_.requestSpeedChange("ego", 3, true);

    std::vector<geometry_msgs::msg::Pose> goal_poses;
    goal_poses.emplace_back(
      api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34579, 40, 0, 0, 0, 0)));
    api_.requestAssignRoute("ego", goal_poses);

    api_.spawn(
      "npc",
      api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34576, 9, 0, 0, 0, 0)),
      getVehicleParameters());
    std::vector<geometry_msgs::msg::Pose> npc_goal_poses;
    npc_goal_poses.emplace_back(
      api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34564, 23, 0, 0, 0, 0)));
    api_.requestAssignRoute("npc", npc_goal_poses);
    api_.setLinearVelocity("npc", 3);
  }

  auto getSampleLaneletPose(const traffic_simulator::LaneletPose & lanelet_pose)
    -> traffic_simulator::CanonicalizedLaneletPose
  {
    return api_.canonicalize(lanelet_pose);
  }
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<cpp_mock_scenarios::SynchronizedAction>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
