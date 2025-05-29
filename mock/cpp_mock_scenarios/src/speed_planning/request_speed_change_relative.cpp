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
class RequestSpeedChangeRelativeScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit RequestSpeedChangeRelativeScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "request_speed_change",
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "private_road_and_walkway_ele_fix/lanelet2_map.osm", __FILE__, false, option),
    request_sent(false)
  {
    start();
  }

private:
  bool request_sent;

  void onUpdate() override
  {
    api_.updateFrame();

    const double current_time = api_.getCurrentTime();
    auto & front_entity = api_.getEntity("front");

    if (current_time >= 0.0 and not request_sent) {
      request_sent = true;
      front_entity.requestSpeedChange(
        traffic_simulator::speed_change::RelativeTargetSpeed(
          "ego", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, 2.0),
      traffic_simulator::speed_change::Transition::linear,
        traffic_simulator::speed_change::Constraint(
          traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, 1.0),
        true);
    }

    const double current_speed = front_entity.getCurrentTwist().linear.x;
    static constexpr double speed_tolerance = 0.05;
    if (current_time >= 0.0 and current_time < 2.0) {
      if (not equals(current_time + 3.0, current_speed, speed_tolerance)) {
        stop(cpp_mock_scenarios::Result::FAILURE);
      }
    } else {
      if (equals(current_speed, 5.0, speed_tolerance)) {
        stop(cpp_mock_scenarios::Result::SUCCESS);
      } else {
        stop(cpp_mock_scenarios::Result::FAILURE);
      }
    }
  }

  void onInitialize() override
  {
    auto & ego_entity = api_.spawn(
      "ego", traffic_simulator::helper::constructCanonicalizedLaneletPose(34741, 0.0, 0.0),
      getVehicleParameters());
    ego_entity.setLinearVelocity(3);
    ego_entity.requestSpeedChange(3.0, true);

    auto & front_entity = api_.spawn(
      "front", traffic_simulator::helper::constructCanonicalizedLaneletPose(34741, 10.0, 0.0),
      getVehicleParameters());
    front_entity.setLinearVelocity(3);
  }
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component =
    std::make_shared<cpp_mock_scenarios::RequestSpeedChangeRelativeScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
