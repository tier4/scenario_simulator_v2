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

#include <cpp_mock_scenarios/catalogs.hpp>
#include <cpp_mock_scenarios/cpp_scenario_node.hpp>
#include <openscenario_msgs/msg/driver_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/api/api.hpp>

// headers in STL
#include <memory>
#include <string>
#include <vector>

class AccelerateAndFollowScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit AccelerateAndFollowScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "accelerate_and_follow", __FILE__, false, option)
  {
    start();
  }

private:
  void onUpdate() override
  {
    double ego_accel = api_.getEntityStatus("ego").action_status.accel.linear.x;
    double ego_twist = api_.getEntityStatus("ego").action_status.twist.linear.x;
    // double npc_accel = api_.getEntityStatus("npc").action_status.accel.linear.x;
    double npc_twist = api_.getEntityStatus("npc").action_status.twist.linear.x;
    // LCOV_EXCL_START
    if (npc_twist > (ego_twist + 1) && ego_accel < 0) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (api_.checkCollision("ego", "npc")) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    // LCOV_EXCL_STOP
    if (api_.getCurrentTime() >= 10) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
  }
  void onInitialize() override
  {
    api_.spawn(false, "ego", getVehicleParameters());
    api_.setEntityStatus(
      "ego", traffic_simulator::helper::constructLaneletPose(34741, 0, 0),
      traffic_simulator::helper::constructActionStatus(3));
    api_.spawn(false, "npc", getVehicleParameters());
    api_.setEntityStatus(
      "npc", traffic_simulator::helper::constructLaneletPose(34741, 10, 0),
      traffic_simulator::helper::constructActionStatus(10));
    api_.setTargetSpeed("npc", 10, true);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<AccelerateAndFollowScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
