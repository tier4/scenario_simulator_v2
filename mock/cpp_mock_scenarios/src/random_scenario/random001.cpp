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

class RandomScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit RandomScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "lanechange_left", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  void onUpdate() override
  {
    const auto spawn_and_change_lane = [&](const auto & entity_name, const auto spawn_s_value) {
      api_.spawn(
        entity_name,
        api_.canonicalize(
          traffic_simulator::helper::constructLaneletPose(34462, spawn_s_value, 0, 0, 0, 0)),
        getVehicleParameters());
      api_.requestSpeedChange(entity_name, 10, true);
      api_.setLinearVelocity(entity_name, 10);
      api_.requestLaneChange(entity_name, 34462);
    };

    if (api_.isInLanelet("ego", 34684, 0.1)) {
      if (!api_.entityExists("lane_following_0")) {
        spawn_and_change_lane("lane_following_0", 0.0);
      }
      if (!api_.entityExists("lane_following_1")) {
        spawn_and_change_lane("lane_following_1", 5.0);
      }
    }
  }
  void onInitialize() override
  {
    api_.spawn(
      "ego",
      api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34621, 10, 0, 0, 0, 0)),
      getVehicleParameters());
    api_.requestAcquirePosition(
      "lane_following_0",
      api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34531, 0, 0, 0, 0, 0)));
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<RandomScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
