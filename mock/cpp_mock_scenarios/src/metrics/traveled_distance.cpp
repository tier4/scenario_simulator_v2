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
#include <cmath>
#include <cpp_mock_scenarios/catalogs.hpp>
#include <cpp_mock_scenarios/cpp_scenario_node.hpp>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/api/api.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <vector>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace cpp_mock_scenarios
{
class TraveledDistanceScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit TraveledDistanceScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "traveled_distance", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "private_road_and_walkway_ele_fix/lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  void onUpdate() override
  {
    // LCOV_EXCL_START
    const auto lanelet_pose = api_.getLaneletPose("ego");
    const auto traveled_distance = api_.getTraveledDistance("ego");
    if (!lanelet_pose) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    const auto difference = std::abs(
      static_cast<traffic_simulator::LaneletPose>(lanelet_pose.value()).s - traveled_distance);
    if (difference > std::numeric_limits<double>::epsilon()) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    // LCOV_EXCL_STOP

    if (api_.getCurrentTime() >= 12) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
  }

  void onInitialize() override
  {
    api_.spawn(
      "ego", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34741, 0, 0)),
      getVehicleParameters());
    api_.setLinearVelocity("ego", 3);
    api_.requestSpeedChange("ego", 3, true);
  }
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<cpp_mock_scenarios::TraveledDistanceScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
