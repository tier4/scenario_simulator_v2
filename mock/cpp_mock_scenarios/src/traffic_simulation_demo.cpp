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
#include <traffic_simulator_msgs/msg/driver_model.hpp>

// headers in STL
#include <memory>
#include <string>
#include <vector>

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
    if (api_.getCurrentTime() >= 4 && api_.entityExists("tom")) {
      api_.despawn("tom");
    }
    if (api_.getCurrentTime() >= 4 && api_.entityExists("obstacle")) {
      api_.setEntityStatus(
        "obstacle", traffic_simulator::helper::constructLaneletPose(120545, 0),
        traffic_simulator::helper::constructActionStatus(10));
    }
    if (api_.getCurrentTime() >= 6 && api_.entityExists("obstacle")) {
      api_.despawn("obstacle");
    }
    if (api_.reachPosition(
          "ego", traffic_simulator::helper::constructLaneletPose(34615, 10.0), 5)) {
      api_.requestAcquirePosition(
        "ego", traffic_simulator::helper::constructLaneletPose(35026, 0.0));
      if (api_.entityExists("npc2")) {
        api_.setTargetSpeed("npc2", 13, true);
      }
    }
    if (api_.reachPosition("ego", traffic_simulator::helper::constructLaneletPose(34579, 0.0), 5)) {
      api_.requestAcquirePosition(
        "ego", traffic_simulator::helper::constructLaneletPose(34675, 0.0));
      if (api_.entityExists("npc2")) {
        api_.setTargetSpeed("npc2", 3, true);
      }
    }
    if (api_.reachPosition(
          "npc2", traffic_simulator::helper::constructLaneletPose(34513, 0.0), 5)) {
      api_.requestAcquirePosition(
        "npc2", traffic_simulator::helper::constructLaneletPose(34630, 0.0));
      api_.setTargetSpeed("npc2", 13, true);
    }
    if (api_.getCurrentTime() > 10.0 && api_.entityExists("bob")) {
      api_.despawn("bob");
    }
  }

  void onInitialize() override
  {
    api_.spawn(false, "ego", getVehicleParameters());
    api_.setEntityStatus(
      "ego", traffic_simulator::helper::constructLaneletPose(120545, 0),
      traffic_simulator::helper::constructActionStatus(10));
    api_.setTargetSpeed("ego", 8, true);
    api_.spawn(false, "tom", getPedestrianParameters());
    api_.setEntityStatus(
      "tom", "ego", traffic_simulator::helper::constructPose(10, 3, 0, 0, 0, -1.57),
      traffic_simulator::helper::constructActionStatus());
    api_.requestWalkStraight("tom");
    api_.setTargetSpeed("tom", 3, true);
    api_.spawn(
      false, "bob", getPedestrianParameters(), "behavior_tree_plugin/PedestrianBehaviorTree",
      traffic_simulator::helper::constructLaneletPose(34378, 0.0),
      traffic_simulator::helper::constructActionStatus(1));
    api_.setTargetSpeed("bob", 1, true);
    api_.spawn(
      false, "npc1", getVehicleParameters(), "behavior_tree_plugin/VehicleBehaviorTree",
      traffic_simulator::helper::constructLaneletPose(34579, 20.0),
      traffic_simulator::helper::constructActionStatus(5));
    api_.setTargetSpeed("npc1", 5, true);
    lanechange_executed_ = false;
    api_.spawn(
      false, "npc2", getVehicleParameters(), "behavior_tree_plugin/VehicleBehaviorTree",
      traffic_simulator::helper::constructLaneletPose(34606, 20.0),
      traffic_simulator::helper::constructActionStatus(5));
    api_.setTargetSpeed("npc2", 0, true);
    api_.requestAssignRoute(
      "ego", std::vector<traffic_simulator_msgs::msg::LaneletPose>{
               traffic_simulator::helper::constructLaneletPose(34675, 0.0),
               traffic_simulator::helper::constructLaneletPose(34690, 0.0)});
    api_.requestAcquirePosition(
      "npc1", traffic_simulator::helper::constructLaneletPose(34675, 0.0));
    api_.spawn(false, "npc3", getVehicleParameters());
    api_.setEntityStatus(
      "npc3", traffic_simulator::helper::constructLaneletPose(34468, 0),
      traffic_simulator::helper::constructActionStatus(10));
    api_.spawn(false, "obstacle", getMiscObjectParameters());
    api_.setEntityStatus(
      "obstacle", "ego", traffic_simulator::helper::constructPose(10, 5, 0, 0, 0, -1.57),
      traffic_simulator::helper::constructActionStatus());
    std::vector<std::pair<double, traffic_simulator::TrafficLightColor>> phase;
    phase = {
      {1, traffic_simulator::TrafficLightColor::GREEN},
      {1, traffic_simulator::TrafficLightColor::YELLOW},
      {1, traffic_simulator::TrafficLightColor::RED}};
    api_.setTrafficLightColorPhase(34802, phase);
  }

private:
  bool lanechange_executed_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<TrafficSimulationDemoScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
