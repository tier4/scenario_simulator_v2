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
class RequestSpeedChangeWithTimeConstraintScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit RequestSpeedChangeWithTimeConstraintScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "request_speed_change_with_time_constraint",
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "private_road_and_walkway_ele_fix/lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  void onUpdate() override
  {
    const auto ego_linear_velocity = api_.getEntity("ego").getCurrentTwist().linear.x;
    if (api_.getCurrentTime() <= 3.9 && ego_linear_velocity > 10.0) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (api_.getCurrentTime() >= 3.999) {
      if (ego_linear_velocity <= 10.0 && ego_linear_velocity >= 9.9) {
        stop(cpp_mock_scenarios::Result::SUCCESS);
      } else {
        stop(cpp_mock_scenarios::Result::FAILURE);
      }
    }
  }

  void onInitialize() override
  {
    api_.spawn(
      "ego", traffic_simulator::helper::constructCanonicalizedLaneletPose(34741, 0.0, 0.0),
      getVehicleParameters());
    auto & ego_entity = api_.getEntity("ego");
    ego_entity.setLinearVelocity(0);
    ego_entity.requestSpeedChange(
      10.0, traffic_simulator::speed_change::Transition::AUTO,
      traffic_simulator::speed_change::Constraint(
        traffic_simulator::speed_change::Constraint::Type::TIME, 4.0),
      false);

    api_.spawn(
      "front", traffic_simulator::helper::constructCanonicalizedLaneletPose(34741, 10.0, 0.0),
      getVehicleParameters());
    auto & front_entity = api_.getEntity("front");
    front_entity.setLinearVelocity(10);
    front_entity.requestSpeedChange(
      10.0, traffic_simulator::speed_change::Transition::LINEAR,
      traffic_simulator::speed_change::Constraint(
        traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, 4.0),
      true);
  }
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component =
    std::make_shared<cpp_mock_scenarios::RequestSpeedChangeWithTimeConstraintScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
