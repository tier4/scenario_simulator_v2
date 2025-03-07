// Copyright 2022 TIER IV, Inc. All rights reserved.
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

// headers in STL
#include <memory>
#include <string>
#include <vector>

class FollowTrajectoryScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit FollowTrajectoryScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "follow_trajectory", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  traffic_simulator_msgs::msg::PolylineTrajectory follow_trajectory;
  int vertex_num = 0;
  Eigen::Vector3d toEigenVector3d(const geometry_msgs::msg::Point & pos)
  {
    return Eigen::Vector3d(pos.x, pos.y, pos.z);
  }
  void onUpdate() override
  {
    const auto t = api_.getCurrentTime();
    const double distance_threshold = 4.0;
    const auto ego_pos = toEigenVector3d(api_.getEntity("ego").getMapPose().position);
    const auto reference_pos =
      toEigenVector3d(follow_trajectory.shape.vertices.at(vertex_num).position.position);

    if ((ego_pos - reference_pos).norm() < distance_threshold) {
      vertex_num++;
      RCLCPP_INFO_STREAM(get_logger(), "passing vertex number: " << vertex_num);
    }

    if (vertex_num >= static_cast<int>(follow_trajectory.shape.vertices.size())) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
    // LCOV_EXCL_STOP
    if (t >= 40) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
  }

  void onInitialize() override
  {
    //Vehicle setting
    api_.spawn(
      "ego", traffic_simulator::helper::constructCanonicalizedLaneletPose(34579, 0, 0),
      getVehicleParameters(), traffic_simulator::VehicleBehavior::contextGamma());
    auto & ego = api_.getEntity("ego");
    ego.setStatus(
      traffic_simulator::helper::constructCanonicalizedLaneletPose(34579, 0, 0),
      traffic_simulator::helper::constructActionStatus(10));
    ego.requestSpeedChange(7, true);

    auto toVertex = [](double time, const geometry_msgs::msg::Pose & pose) {
      traffic_simulator_msgs::msg::Vertex vertex;
      vertex.position = pose;
      vertex.time = time;
      return vertex;
    };
    follow_trajectory.shape.vertices.push_back(toVertex(
      0.0,
      traffic_simulator::pose::toMapPose(
        traffic_simulator::helper::constructCanonicalizedLaneletPose(34579, 0.0, 0, 0, 0, 0))));
    follow_trajectory.shape.vertices.push_back(toVertex(
      10.0,
      traffic_simulator::pose::toMapPose(
        traffic_simulator::helper::constructCanonicalizedLaneletPose(34579, 10.0, 0, 0, 0, 0))));
    follow_trajectory.shape.vertices.push_back(toVertex(
      12.0,
      traffic_simulator::pose::toMapPose(
        traffic_simulator::helper::constructCanonicalizedLaneletPose(34600, 30.0, 0, 0, 0, 0))));
    follow_trajectory.shape.vertices.push_back(toVertex(
      13.0,
      traffic_simulator::pose::toMapPose(
        traffic_simulator::helper::constructCanonicalizedLaneletPose(34600, 20.0, 0, 0, 0, 0))));
    follow_trajectory.shape.vertices.push_back(toVertex(
      14.0,
      traffic_simulator::pose::toMapPose(
        traffic_simulator::helper::constructCanonicalizedLaneletPose(34579, 40.0, 0, 0, 0, 0))));
    follow_trajectory.shape.vertices.push_back(toVertex(
      25.0,
      traffic_simulator::pose::toMapPose(
        traffic_simulator::helper::constructCanonicalizedLaneletPose(34579, 50.0, 0, 0, 0, 0))));

    follow_trajectory.initial_distance_offset = 0.0;
    follow_trajectory.dynamic_constraints_ignorable = true;
    follow_trajectory.base_time = 5.0;
    follow_trajectory.closed = false;
    std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> follow_trajectory_ptr =
      std::make_shared<traffic_simulator_msgs::msg::PolylineTrajectory>(follow_trajectory);
    ego.requestFollowTrajectory(follow_trajectory_ptr);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<FollowTrajectoryScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
