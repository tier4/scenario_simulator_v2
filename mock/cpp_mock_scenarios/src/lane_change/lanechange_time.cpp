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
class LaneChangeTimeScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit LaneChangeTimeScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "lanechange_left", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  int lanechange_frames = 0;
  void onUpdate() override
  {
    const auto & ego_entity = api_.getEntity("ego");
    if (ego_entity.getCurrentAction() == "lane_change") {
      lanechange_frames++;
    }
    if (ego_entity.getCurrentAction() != "lane_change" && api_.getCurrentTime() >= 10.0) {
      if (const double step_time = 50e-3, expected_time = 20.0,
          real_time = static_cast<double>(lanechange_frames) * step_time;
          expected_time - step_time <= real_time && real_time <= expected_time + step_time) {
        stop(cpp_mock_scenarios::Result::SUCCESS);
      } else {
        stop(cpp_mock_scenarios::Result::FAILURE);
      }
    }
    // LCOV_EXCL_START
    if (api_.getCurrentTime() >= 30.0) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    // LCOV_EXCL_STOP
  }
  void onInitialize() override
  {
    api_.spawn(
      "ego", traffic_simulator::helper::constructCanonicalizedLaneletPose(34462, 10.0, 0.0),
      getVehicleParameters());
    auto & ego_entity = api_.getEntity("ego");
    ego_entity.setLinearVelocity(10);
    ego_entity.requestSpeedChange(10, true);
    ego_entity.requestLaneChange(
      traffic_simulator::lane_change::RelativeTarget(
        "ego", traffic_simulator::lane_change::Direction::LEFT, 1, 0),
      traffic_simulator::lane_change::TrajectoryShape::CUBIC,
      traffic_simulator::lane_change::Constraint(
        traffic_simulator::lane_change::Constraint::Type::TIME, 20.0));
  }
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<cpp_mock_scenarios::LaneChangeTimeScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
