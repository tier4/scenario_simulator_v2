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

class HeadOnPedestrianScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit HeadOnPedestrianScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "head_on_pedestrian",
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map", "lanelet2_map.osm",
      __FILE__, false, option)
  {
    start();
  }

private:
  Eigen::Vector3d toEigenVector3d(const geometry_msgs::msg::Point & pos)
  {
    return Eigen::Vector3d(pos.x, pos.y, pos.z);
  }
  std::vector<Eigen::Vector3d> init_pos;
  void onUpdate() override
  {
    const auto t = api_.getCurrentTime();
    float goal_threshold = 1.5;
    auto & ego = api_.getEntity("ego");
    auto & ego2 = api_.getEntity("ego2");
    auto & bob = api_.getEntity("bob");
    auto & bob2 = api_.getEntity("bob2");
    if ((init_pos[0] - toEigenVector3d(ego.getMapPose().position)).norm() < goal_threshold) {
      ego.requestSpeedChange(0, true);
    }
    if ((init_pos[1] - toEigenVector3d(ego2.getMapPose().position)).norm() < goal_threshold) {
      ego2.requestSpeedChange(0, true);
    }
    if ((init_pos[2] - toEigenVector3d(bob.getMapPose().position)).norm() < goal_threshold) {
      bob.requestSpeedChange(0, true);
    }
    if ((init_pos[3] - toEigenVector3d(bob2.getMapPose().position)).norm() < goal_threshold) {
      bob2.requestSpeedChange(0, true);
    }
    if (
      (init_pos[0] - toEigenVector3d(ego.getMapPose().position)).norm() < goal_threshold and
      (init_pos[1] - toEigenVector3d(ego2.getMapPose().position)).norm() < goal_threshold and
      (init_pos[2] - toEigenVector3d(bob.getMapPose().position)).norm() < goal_threshold and
      (init_pos[3] - toEigenVector3d(bob2.getMapPose().position)).norm() < goal_threshold) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
    // LCOV_EXCL_STOP
    if (t >= 60) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
  }

  void onInitialize() override
  {
    auto toVertex = [](double time, const geometry_msgs::msg::Pose & pose) {
      traffic_simulator_msgs::msg::Vertex vertex;
      vertex.position = pose;
      vertex.time = time;
      return vertex;
    };
    auto toTrajectory =
      [&](const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & goal_pose) {
        traffic_simulator_msgs::msg::PolylineTrajectory follow_trajectory;
        follow_trajectory.shape.vertices.push_back(toVertex(0.0, start_pose));
        follow_trajectory.shape.vertices.push_back(toVertex(10.0, goal_pose));
        follow_trajectory.initial_distance_offset = 0.0;
        follow_trajectory.dynamic_constraints_ignorable = true;
        follow_trajectory.base_time = 0.0;
        follow_trajectory.closed = false;
        return follow_trajectory;
      };
    //Pedestrian1 setting
    api_.spawn(
      "bob", traffic_simulator::helper::constructCanonicalizedLaneletPose(34378, 0.0, -1),
      getPedestrianParameters(), traffic_simulator::PedestrianBehavior::contextGamma());
    auto & bob = api_.getEntity("bob");
    bob.requestSpeedChange(
      0.5, traffic_simulator::speed_change::Transition::LINEAR,
      traffic_simulator::speed_change::Constraint(
        traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, 1.0),
      true);
    std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> follow_trajectory_ptr1 =
      std::make_shared<traffic_simulator_msgs::msg::PolylineTrajectory>(toTrajectory(
        traffic_simulator::pose::toMapPose(
          traffic_simulator::helper::constructCanonicalizedLaneletPose(34378, 0.0, -1, 0, 0, 0)),
        traffic_simulator::pose::toMapPose(
          traffic_simulator::helper::constructCanonicalizedLaneletPose(34378, 7.5, 1, 0, 0, 0))));
    bob.requestFollowTrajectory(follow_trajectory_ptr1);
    //Pedestrian2 setting
    api_.spawn(
      "bob2", traffic_simulator::helper::constructCanonicalizedLaneletPose(34378, 0.0, 1),
      getPedestrianParameters(), traffic_simulator::PedestrianBehavior::contextGamma());
    auto & bob2 = api_.getEntity("bob2");
    bob2.requestSpeedChange(
      0.5, traffic_simulator::speed_change::Transition::LINEAR,
      traffic_simulator::speed_change::Constraint(
        traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, 1.0),
      true);
    std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> follow_trajectory_ptr2 =
      std::make_shared<traffic_simulator_msgs::msg::PolylineTrajectory>(toTrajectory(
        traffic_simulator::pose::toMapPose(
          traffic_simulator::helper::constructCanonicalizedLaneletPose(34378, 0.0, 1, 0, 0, 0)),
        traffic_simulator::pose::toMapPose(
          traffic_simulator::helper::constructCanonicalizedLaneletPose(34378, 7.5, -1, 0, 0, 0))));
    bob2.requestFollowTrajectory(follow_trajectory_ptr2);

    //Pedestrian3 setting
    api_.spawn(
      "ego", traffic_simulator::helper::constructCanonicalizedLaneletPose(34378, 7.5, 1),
      getPedestrianParameters(), traffic_simulator::PedestrianBehavior::contextGamma());
    auto & ego = api_.getEntity("ego");
    ego.requestSpeedChange(
      0.5, traffic_simulator::speed_change::Transition::LINEAR,
      traffic_simulator::speed_change::Constraint(
        traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, 1.0),
      true);
    std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> follow_trajectory_ptr3 =
      std::make_shared<traffic_simulator_msgs::msg::PolylineTrajectory>(toTrajectory(
        traffic_simulator::pose::toMapPose(
          traffic_simulator::helper::constructCanonicalizedLaneletPose(34378, 7.5, 1, 0, 0, 0)),
        traffic_simulator::pose::toMapPose(
          traffic_simulator::helper::constructCanonicalizedLaneletPose(34378, 0.0, -1, 0, 0, 0))));
    ego.requestFollowTrajectory(follow_trajectory_ptr3);

    //Pedestrian4 setting
    api_.spawn(
      "ego2", traffic_simulator::helper::constructCanonicalizedLaneletPose(34378, 7.5, -1),
      getPedestrianParameters(), traffic_simulator::PedestrianBehavior::contextGamma());
    auto & ego2 = api_.getEntity("ego2");
    ego2.requestSpeedChange(
      0.5, traffic_simulator::speed_change::Transition::LINEAR,
      traffic_simulator::speed_change::Constraint(
        traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, 1.0),
      true);
    std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> follow_trajectory_ptr4 =
      std::make_shared<traffic_simulator_msgs::msg::PolylineTrajectory>(toTrajectory(
        traffic_simulator::pose::toMapPose(
          traffic_simulator::helper::constructCanonicalizedLaneletPose(34378, 7.5, -1, 0, 0, 0)),
        traffic_simulator::pose::toMapPose(
          traffic_simulator::helper::constructCanonicalizedLaneletPose(34378, 0.0, 1, 0, 0, 0))));
    ego2.requestFollowTrajectory(follow_trajectory_ptr4);

    init_pos.emplace_back(toEigenVector3d(bob.getMapPose().position));
    init_pos.emplace_back(toEigenVector3d(bob2.getMapPose().position));
    init_pos.emplace_back(toEigenVector3d(ego.getMapPose().position));
    init_pos.emplace_back(toEigenVector3d(ego2.getMapPose().position));
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<HeadOnPedestrianScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
