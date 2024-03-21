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

namespace cpp_mock_scenarios
{
class FollowPolylineTrajectoryWithDoNothingPluginScenario
: public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit FollowPolylineTrajectoryWithDoNothingPluginScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "lanechange_left", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  bool requested = false;
  void onUpdate() override
  {
    // LCOV_EXCL_START
    if (api_.getCurrentTime() >= 10.0) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    // LCOV_EXCL_STOP
  }
  void onInitialize() override
  {
    api_.spawn(
      "ego",
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0))
        .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1)),
      getVehicleParameters(),
      traffic_simulator::entity::VehicleEntity::BuiltinBehavior::doNothing());
    api_.setLinearVelocity("ego", 10);
    api_.requestSpeedChange("ego", 10, true);
    api_.requestFollowTrajectory(
      "ego", std::make_shared<traffic_simulator_msgs::msg::PolylineTrajectory>(
               traffic_simulator_msgs::build<traffic_simulator_msgs::msg::PolylineTrajectory>()
                 .initial_distance_offset(0.0)
                 .dynamic_constraints_ignorable(false)
                 .base_time(0.0)
                 .closed(false)
                 .shape({})));
  }
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component =
    std::make_shared<cpp_mock_scenarios::FollowPolylineTrajectoryWithDoNothingPluginScenario>(
      options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}