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
#include <traffic_simulator_msgs/msg/polyline.hpp>
#include <traffic_simulator_msgs/msg/vertex.hpp>
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
      "lanelet2_map.osm", __FILE__, false, option),
    spawn_pose(
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0))
        .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1))),
    trajectory_start_pose(
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(10).y(0).z(0))
        .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1))),
    trajectory_waypoint_pose(
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(15).y(0).z(0))
        .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1))),
    trajectory_goal_pose(
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(20).y(0).z(0))
        .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1)))
  {
    start();
  }

private:
  bool requested = false;
  const geometry_msgs::msg::Pose spawn_pose;
  const geometry_msgs::msg::Pose trajectory_start_pose;
  const geometry_msgs::msg::Pose trajectory_waypoint_pose;
  const geometry_msgs::msg::Pose trajectory_goal_pose;

  void onUpdate() override
  {
    // LCOV_EXCL_START
    if (api_.getCurrentTime() >= 10.0) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    // LCOV_EXCL_STOP
    if (equals(api_.getCurrentTime(), 0.0, 0.01) && !api_.reachPosition("ego", spawn_pose, 0.1)) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (
      equals(api_.getCurrentTime(), 1.0, 0.01) &&
      !api_.reachPosition("ego", trajectory_start_pose, 0.1)) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (
      equals(api_.getCurrentTime(), 1.5, 0.01) &&
      !api_.reachPosition("ego", trajectory_waypoint_pose, 0.1)) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (
      equals(api_.getCurrentTime(), 2.0, 0.01) &&
      !api_.reachPosition("ego", trajectory_goal_pose, 0.1)) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (equals(api_.getCurrentTime(), 2.5, 0.01)) {
      if (api_.reachPosition("ego", trajectory_goal_pose, 0.1)) {
        stop(cpp_mock_scenarios::Result::SUCCESS);
      } else {
        stop(cpp_mock_scenarios::Result::FAILURE);
      }
    }
  }
  void onInitialize() override
  {
    api_.spawn(
      "ego", spawn_pose, getVehicleParameters(),
      traffic_simulator::entity::VehicleEntity::BuiltinBehavior::doNothing());
    api_.setLinearVelocity("ego", 10);
    api_.requestSpeedChange("ego", 10, true);
    api_.requestFollowTrajectory(
      "ego",
      std::make_shared<traffic_simulator_msgs::msg::PolylineTrajectory>(
        traffic_simulator_msgs::build<traffic_simulator_msgs::msg::PolylineTrajectory>()
          .initial_distance_offset(0.0)
          .dynamic_constraints_ignorable(true)
          .base_time(0.0)
          .closed(false)
          .shape(traffic_simulator_msgs::build<traffic_simulator_msgs::msg::Polyline>().vertices(
            [this]() {
              std::vector<traffic_simulator_msgs::msg::Vertex> vertices;
              vertices.emplace_back(
                traffic_simulator_msgs::build<traffic_simulator_msgs::msg::Vertex>()
                  .time(1.0)
                  .position(trajectory_start_pose));
              vertices.emplace_back(
                traffic_simulator_msgs::build<traffic_simulator_msgs::msg::Vertex>()
                  .time(1.5)
                  .position(trajectory_waypoint_pose));
              vertices.emplace_back(
                traffic_simulator_msgs::build<traffic_simulator_msgs::msg::Vertex>()
                  .time(2.0)
                  .position(trajectory_goal_pose));
              return vertices;
            }()))));
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
