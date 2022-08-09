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
#include <traffic_simulator_msgs/msg/driver_model.hpp>

// headers in STL
#include <memory>
#include <string>
#include <vector>

class RequestSpeedChangeScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit RequestSpeedChangeScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "request_speed_change",
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "private_road_and_walkway_ele_fix/lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  bool speed_reached;
  void onUpdate() override
  {
    /**
     * @brief checking linear acceleration
     */
    if (
      api_.getCurrentTime() != 0.0 && api_.getCurrentTime() <= 1.0 &&
      api_.getEntityStatus("ego").action_status.accel.linear.x != 10.0) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (
      api_.getCurrentTime() >= 1.05 &&
      api_.getEntityStatus("ego").action_status.accel.linear.x > 3.0) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }

    /**
     * @brief checking linear speed
     */
    if (
      api_.getCurrentTime() <= 0.9 &&
      api_.getEntityStatus("ego").action_status.twist.linear.x > 10.0) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (
      api_.getCurrentTime() >= 1.0 &&
      api_.getEntityStatus("ego").action_status.twist.linear.x <= 10.0) {
      speed_reached = true;
    }
    if (
      speed_reached && api_.getCurrentTime() >= 1.5 &&
      api_.getEntityStatus("ego").action_status.twist.linear.x >= 13.88) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
  }

  void onInitialize() override
  {
    speed_reached = false;
    api_.spawn("ego", getVehicleParameters());
    api_.setEntityStatus(
      "ego", traffic_simulator::helper::constructLaneletPose(34741, 0, 0),
      traffic_simulator::helper::constructActionStatus(0));
    api_.requestSpeedChange(
      "ego", 10.0, traffic_simulator::speed_change::Transition::LINEAR,
      traffic_simulator::speed_change::Constraint(
        traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, 10.0),
      false);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<RequestSpeedChangeScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
