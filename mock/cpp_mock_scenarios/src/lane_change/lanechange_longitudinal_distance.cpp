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
class LaneChangeLongitudinalDistanceScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit LaneChangeLongitudinalDistanceScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "lanechange_left", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  bool requested = false;
  bool lanechange_finished = false;
  void onUpdate() override
  {
    if (api_.isInLanelet("ego", 34513, 0.1)) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
  }
  void onInitialize() override
  {
    api_.spawn(
      "ego",
      api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34462, 10, 0, 0, 0, 0)),
      getVehicleParameters());
    api_.setLinearVelocity("ego", 1);
    api_.requestSpeedChange("ego", 1, true);
    api_.requestLaneChange(
      "ego",
      traffic_simulator::lane_change::RelativeTarget(
        "ego", traffic_simulator::lane_change::Direction::LEFT, 1, 0),
      traffic_simulator::lane_change::TrajectoryShape::CUBIC,
      traffic_simulator::lane_change::Constraint(
        traffic_simulator::lane_change::Constraint::Type::LONGITUDINAL_DISTANCE, 3.0));
  }
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component =
    std::make_shared<cpp_mock_scenarios::LaneChangeLongitudinalDistanceScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
