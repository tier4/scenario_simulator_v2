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
#include <openscenario_msgs/msg/driver_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/api/api.hpp>

// headers in STL
#include <memory>
#include <string>
#include <vector>

class CancelRequest : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit CancelRequest(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "idiot_npc", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  bool canceled = false;
  void onUpdate() override
  {
    if (api_.reachPosition(
          "ego", traffic_simulator::helper::constructLaneletPose(34513, 30, 0, 0, 0, 0), 3.0)) {
      api_.cancelRequest("ego");
      canceled = true;
    }
    if (api_.isInLanelet("ego", 34507, 0.1)) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
  }
  void onInitialize() override
  {
    api_.spawn(false, "ego", getVehicleParameters());
    api_.setEntityStatus(
      "ego", traffic_simulator::helper::constructLaneletPose(34513, 0, 0, 0, 0, 0),
      traffic_simulator::helper::constructActionStatus(7));
    api_.setTargetSpeed("ego", 7, true);
    const geometry_msgs::msg::Pose goal_pose =
      api_.toMapPose(traffic_simulator::helper::constructLaneletPose(34408, 0, 0, 0, 0, 0));
    api_.requestAcquirePosition("ego", goal_pose);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<CancelRequest>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
