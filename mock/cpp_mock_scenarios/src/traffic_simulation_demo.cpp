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
class TrafficSimulationDemoScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit TrafficSimulationDemoScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "traffic_simulation_demo",
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map", "lanelet2_map.osm",
      __FILE__, false, option)
  {
    start();
  }

private:
  void onUpdate() override
  {
    if (api_.getCurrentTime() >= 60) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
    if (api_.getCurrentTime() >= 4 && api_.isEntityExist("tom")) {
      api_.despawn("tom");
    }
    if (api_.getCurrentTime() >= 4 && api_.isEntityExist("obstacle")) {
      api_.getEntity("obstacle")
        .setStatus(
          traffic_simulator::helper::constructCanonicalizedLaneletPose(120545, 0.0, 0.0),
          traffic_simulator::helper::constructActionStatus(10));
    }
    if (api_.getCurrentTime() >= 6 && api_.isEntityExist("obstacle")) {
      api_.despawn("obstacle");
    }
    auto & ego_entity = api_.getEntity("ego");
    auto & npc2_entity = api_.getEntity("npc2");
    if (ego_entity.isNearbyPosition(
          traffic_simulator::helper::constructCanonicalizedLaneletPose(34615, 10.0, 0.0), 5)) {
      ego_entity.requestAcquirePosition(
        traffic_simulator::helper::constructCanonicalizedLaneletPose(35026, 0.0, 0.0));
      if (api_.isEntityExist("npc2")) {
        npc2_entity.requestSpeedChange(13, true);
      }
    }
    if (ego_entity.isNearbyPosition(
          traffic_simulator::helper::constructCanonicalizedLaneletPose(34579, 0.0, 0.0), 5)) {
      ego_entity.requestAcquirePosition(
        traffic_simulator::helper::constructCanonicalizedLaneletPose(34675, 0.0, 0.0));
      if (api_.isEntityExist("npc2")) {
        npc2_entity.requestSpeedChange(3, true);
      }
    }
    if (npc2_entity.isNearbyPosition(
          traffic_simulator::helper::constructCanonicalizedLaneletPose(34513, 0.0, 0.0), 5)) {
      npc2_entity.requestAcquirePosition(
        traffic_simulator::helper::constructCanonicalizedLaneletPose(34630, 0.0, 0.0));
      npc2_entity.requestSpeedChange(13, true);
    }
    if (api_.getCurrentTime() > 10.0 && api_.isEntityExist("bob")) {
      api_.despawn("bob");
    }
  }

  void onInitialize() override
  {
    lanechange_executed_ = false;

    auto & ego_entity = api_.spawn(
      "ego", traffic_simulator::helper::constructCanonicalizedLaneletPose(120545, 0.0, 0.0),
      getVehicleParameters());
    ego_entity.setLinearVelocity(10);
    ego_entity.requestSpeedChange(8, true);
    ego_entity.requestAssignRoute(std::vector<traffic_simulator::CanonicalizedLaneletPose>{
      traffic_simulator::helper::constructCanonicalizedLaneletPose(34675, 0.0, 0.0),
      traffic_simulator::helper::constructCanonicalizedLaneletPose(34690, 0.0, 0.0)});

    auto & tom_entity = api_.spawn(
      "tom", traffic_simulator::helper::constructPose(10, 3, 0, 0, 0, -1.57),
      getPedestrianParameters());
    tom_entity.setStatus(
      ego_entity.getMapPose(), traffic_simulator::helper::constructPose(10, 3, 0, 0, 0, -1.57),
      traffic_simulator::helper::constructActionStatus());
    tom_entity.requestWalkStraight();
    tom_entity.requestSpeedChange(3, true);

    auto & bob_entity = api_.spawn(
      "bob", traffic_simulator::helper::constructCanonicalizedLaneletPose(34378, 0.0, 0.0),
      getPedestrianParameters());
    bob_entity.setLinearVelocity(1.0);
    bob_entity.requestSpeedChange(1, true);

    auto & npc1_entity = api_.spawn(
      "npc1", traffic_simulator::helper::constructCanonicalizedLaneletPose(34579, 20.0, 0.0),
      getVehicleParameters());
    npc1_entity.setLinearVelocity(5.0);
    npc1_entity.requestSpeedChange(5, true);
    npc1_entity.requestAcquirePosition(
      traffic_simulator::helper::constructCanonicalizedLaneletPose(34675, 0.0, 0.0));

    auto & npc2_entity = api_.spawn(
      "npc2", traffic_simulator::helper::constructCanonicalizedLaneletPose(34606, 20.0, 0.0),
      getVehicleParameters());
    npc2_entity.setLinearVelocity(5);
    npc2_entity.requestSpeedChange(0, true);

    auto & npc3_entity = api_.spawn(
      "npc3", traffic_simulator::helper::constructCanonicalizedLaneletPose(34468, 0.0, 0.0),
      getVehicleParameters());
    npc3_entity.setLinearVelocity(10);

    auto & obstacle_entity = api_.spawn(
      "obstacle", traffic_simulator::helper::constructPose(10, 5, 0, 0, 0, -1.57),
      getMiscObjectParameters());
    obstacle_entity.setStatus(
      ego_entity.getMapPose(), traffic_simulator::helper::constructPose(10, 5, 0, 0, 0, -1.57),
      traffic_simulator::helper::constructActionStatus());

    api_.getConventionalTrafficLights()->setTrafficLightsColor(
      34802, traffic_simulator::TrafficLight::Color::green);
  }

private:
  bool lanechange_executed_;
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<cpp_mock_scenarios::TrafficSimulationDemoScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
