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
    if (api_.getCurrentTime() >= 4 && api_.entityExists("tom")) {
      api_.despawn("tom");
    }
    if (api_.getCurrentTime() >= 4 && api_.entityExists("obstacle")) {
      api_.setEntityStatus(
        "obstacle", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(120545, 0)),
        traffic_simulator::helper::constructActionStatus(10));
    }
    if (api_.getCurrentTime() >= 6 && api_.entityExists("obstacle")) {
      api_.despawn("obstacle");
    }
    if (api_.reachPosition(
          "ego", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34615, 10.0)),
          5)) {
      api_.requestAcquirePosition(
        "ego", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(35026, 0.0)));
      if (api_.entityExists("npc2")) {
        api_.requestSpeedChange("npc2", 13, true);
      }
    }
    if (api_.reachPosition(
          "ego", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34579, 0.0)),
          5)) {
      api_.requestAcquirePosition(
        "ego", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34675, 0.0)));
      if (api_.entityExists("npc2")) {
        api_.requestSpeedChange("npc2", 3, true);
      }
    }
    if (api_.reachPosition(
          "npc2", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34513, 0.0)),
          5)) {
      api_.requestAcquirePosition(
        "npc2", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34630, 0.0)));
      api_.requestSpeedChange("npc2", 13, true);
    }
    if (api_.getCurrentTime() > 10.0 && api_.entityExists("bob")) {
      api_.despawn("bob");
    }
  }

  void onInitialize() override
  {
    lanechange_executed_ = false;

    api_.spawn(
      "ego", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(120545, 0)),
      getVehicleParameters());
    api_.setLinearVelocity("ego", 10);
    api_.requestSpeedChange("ego", 8, true);
    api_.requestAssignRoute(
      "ego", std::vector<traffic_simulator::CanonicalizedLaneletPose>{
               api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34675, 0.0)),
               api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34690, 0.0))});

    api_.spawn(
      "tom", traffic_simulator::helper::constructPose(10, 3, 0, 0, 0, -1.57),
      getPedestrianParameters());
    api_.setEntityStatus(
      "tom", "ego", traffic_simulator::helper::constructPose(10, 3, 0, 0, 0, -1.57),
      traffic_simulator::helper::constructActionStatus());
    api_.requestWalkStraight("tom");
    api_.requestSpeedChange("tom", 3, true);

    api_.spawn(
      "bob", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34378, 0.0)),
      getPedestrianParameters());
    api_.setLinearVelocity("bob", 1.0);
    api_.requestSpeedChange("bob", 1, true);

    api_.spawn(
      "npc1", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34579, 20.0)),
      getVehicleParameters());
    api_.setLinearVelocity("npc1", 5.0);
    api_.requestSpeedChange("npc1", 5, true);
    api_.requestAcquirePosition(
      "npc1", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34675, 0.0)));

    api_.spawn(
      "npc2", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34606, 20.0)),
      getVehicleParameters());
    api_.setLinearVelocity("npc2", 5);
    api_.requestSpeedChange("npc2", 0, true);

    api_.spawn(
      "npc3", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34468, 0)),
      getVehicleParameters());
    api_.setLinearVelocity("npc3", 10);

    api_.spawn(
      "obstacle", traffic_simulator::helper::constructPose(10, 5, 0, 0, 0, -1.57),
      getMiscObjectParameters());
    api_.setEntityStatus(
      "obstacle", "ego", traffic_simulator::helper::constructPose(10, 5, 0, 0, 0, -1.57),
      traffic_simulator::helper::constructActionStatus());

    api_.getConventionalTrafficLight(34802).emplace(traffic_simulator::TrafficLight::Color::green);
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
