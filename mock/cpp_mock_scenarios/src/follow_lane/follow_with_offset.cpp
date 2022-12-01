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

class FollowLaneWithOffset : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit FollowLaneWithOffset(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "follow_lane_with_offset",
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map", "lanelet2_map.osm",
      __FILE__, false, option)
  {
    start();
  }

private:
  bool requested = false;
  void onUpdate() override
  {
    if (api_.isInLanelet("ego", 34507, 0.1)) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
    const auto lanelet_pose = api_.getLaneletPose("ego");
    if (!lanelet_pose || std::abs(lanelet_pose.get().offset) <= 2.8) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
  }
  void onInitialize() override
  {
    api_.spawn(
      "ego", traffic_simulator::helper::constructLaneletPose(34513, 0, 3, 0, 0, 0),
      getVehicleParameters());
    api_.setLinearVelocity("ego", 10);
    api_.requestSpeedChange("ego", 10, true);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<FollowLaneWithOffset>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
