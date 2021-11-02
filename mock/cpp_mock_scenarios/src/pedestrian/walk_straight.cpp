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

class StopAtCrosswalkScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit StopAtCrosswalkScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "stop_at_crosswalk", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  void onUpdate() override
  {
    const auto t = api_.getCurrentTime();
    // LCOV_EXCL_START
    if (api_.entityExists("bob") && api_.checkCollision("ego", "bob")) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (t >= 6.1 && 6.2 >= t) {
      const auto vel = api_.getEntityStatus("ego").action_status.twist.linear.x;
      if (std::fabs(0.01) <= vel) {
        stop(cpp_mock_scenarios::Result::FAILURE);
      }
    }
    // LCOV_EXCL_STOP
    if (t >= 10) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
  }

  void onInitialize() override
  {
    api_.spawn(false, "ego", getVehicleParameters());
    api_.setEntityStatus(
      "ego", traffic_simulator::helper::constructLaneletPose(120545, 0),
      traffic_simulator::helper::constructActionStatus(10));
    api_.setTargetSpeed("ego", 8, true);
    api_.requestAssignRoute(
      "ego", std::vector<traffic_simulator_msgs::msg::LaneletPose>{
               traffic_simulator::helper::constructLaneletPose(34675, 0.0),
               traffic_simulator::helper::constructLaneletPose(34690, 0.0)});

    api_.spawn(false, "bob", getPedestrianParameters());
    api_.setEntityStatus(
      "bob", traffic_simulator::helper::constructLaneletPose(34378, 0.0),
      traffic_simulator::helper::constructActionStatus(1));
    api_.requestWalkStraight("bob");
  }

private:
  bool lanechange_executed_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<StopAtCrosswalkScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
