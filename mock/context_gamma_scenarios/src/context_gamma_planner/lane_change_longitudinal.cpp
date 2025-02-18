// Copyright 2022 TIER IV, Inc. All rights reserved.
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

// headers in STL
#include <memory>
#include <string>
#include <vector>

class LaneChangeLongitudinalScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit LaneChangeLongitudinalScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "lane_change_longitudinal",
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map", "lanelet2_map.osm",
      __FILE__, false, option)
  {
    start();
  }

private:
  void onUpdate() override
  {
    const auto t = api_.getCurrentTime();
    auto & ego = api_.getEntity("ego");
    if (1 < t && t < 1.1) {
      ego.requestLaneChange(
        traffic_simulator::lane_change::RelativeTarget(
          "ego", traffic_simulator::lane_change::Direction::RIGHT, 1, 0.0),
        traffic_simulator::lane_change::TrajectoryShape::CUBIC,
        traffic_simulator::lane_change::Constraint(
          traffic_simulator::lane_change::Constraint::Type::LONGITUDINAL_DISTANCE, 20,
          traffic_simulator::lane_change::Constraint::Policy::BEST_EFFORT));
    }
    if (const auto & lanelet_pose = ego.getCanonicalizedLaneletPose()) {
      if (
        lanelet_pose.value().getLaneletPose().lanelet_id == 34462 and
        lanelet_pose.value().getLaneletPose().s >= 28.0) {
        stop(cpp_mock_scenarios::Result::SUCCESS);
      }
    }
    // LCOV_EXCL_STOP
    if (t >= 30) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
  }

  void onInitialize() override
  {
    //Vehicle setting
    api_.spawn(
      "ego", traffic_simulator::helper::constructCanonicalizedLaneletPose(34513, 0, 0),
      getVehicleParameters(), traffic_simulator::VehicleBehavior::contextGamma());
    auto & ego = api_.getEntity("ego");
    ego.requestSpeedChange(7, true);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<LaneChangeLongitudinalScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
