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
#include <context_gamma_scenarios/catalogs.hpp>
#include <context_gamma_scenarios/context_gamma_scenario_node.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/api/api.hpp>
#include <vector>
class ReverseWalkAtRoadScenario : public context_gamma_scenarios::ContextGammaScenarioNode
{
public:
  explicit ReverseWalkAtRoadScenario(const rclcpp::NodeOptions & option)
  : context_gamma_scenarios::ContextGammaScenarioNode(
      "reverse_walk_at_road",
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map", "lanelet2_map.osm",
      __FILE__, false, option)
  {
    start();
  }

private:
  const geometry_msgs::msg::Pose goal_pose_ =
    traffic_simulator::pose::toMapPose(traffic_simulator::helper::constructCanonicalizedLaneletPose(
      34976, 2, 0, 0, 0, 0, api_.getHdmapUtils()));
  void onUpdate() override
  {
    const auto t = api_.getCurrentTime();
    if (6.5 < t && t < 7) {
      api_.requestSpeedChange("ego", 0, true);
    }
    const Eigen::Vector3d target_pose{goal_pose_.position.x, goal_pose_.position.y, 0};
    const Eigen::Vector3d current_pose{
      api_.getEntityStatus("bob").getMapPose().position.x,
      api_.getEntityStatus("bob").getMapPose().position.y, 0};
    if ((target_pose - current_pose).norm() < 2.0) {
      stop(context_gamma_scenarios::Result::SUCCESS);
    }
    // LCOV_EXCL_STOP
    if (t >= 120) {
      stop(context_gamma_scenarios::Result::FAILURE);
    }
  }

  void onInitialize() override
  {
    api_.spawn(
      "ego",
      traffic_simulator::helper::constructCanonicalizedLaneletPose(
        34976, 0, 0, api_.getHdmapUtils()),
      getVehicleParameters(), traffic_simulator::VehicleBehavior::contextGamma());
    api_.setEntityStatus(
      "ego",
      traffic_simulator::helper::constructCanonicalizedLaneletPose(
        34976, 0, 0, api_.getHdmapUtils()),
      traffic_simulator::helper::constructActionStatus());
    api_.requestAssignRoute(
      "ego", std::vector<traffic_simulator::CanonicalizedLaneletPose>{
               traffic_simulator::helper::constructCanonicalizedLaneletPose(
                 34591, 0.0, 0, api_.getHdmapUtils()),
               traffic_simulator::helper::constructCanonicalizedLaneletPose(
                 34690, 0.0, 0, api_.getHdmapUtils()),
             });
    api_.requestSpeedChange("ego", 7, true);

    api_.spawn(
      "bob",
      traffic_simulator::helper::constructCanonicalizedLaneletPose(
        34981, 0.0, 0, api_.getHdmapUtils()),
      getPedestrianParameters(), traffic_simulator::PedestrianBehavior::contextGamma());
    api_.requestSpeedChange(
      "bob", 1, traffic_simulator::speed_change::Transition::LINEAR,
      traffic_simulator::speed_change::Constraint(
        traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, 1.0),
      true);

    auto toVertex = [](double time, const geometry_msgs::msg::Pose & pose) {
      traffic_simulator_msgs::msg::Vertex vertex;
      vertex.position = pose;
      vertex.time = time;
      return vertex;
    };
    traffic_simulator_msgs::msg::PolylineTrajectory follow_trajectory;
    follow_trajectory.shape.vertices.push_back(toVertex(
      0.0, traffic_simulator::pose::toMapPose(
             traffic_simulator::helper::constructCanonicalizedLaneletPose(
               34981, 0, 0, 0, 0, 0, api_.getHdmapUtils()))));
    follow_trajectory.shape.vertices.push_back(toVertex(30.0, goal_pose_));

    follow_trajectory.initial_distance_offset = 0.0;
    follow_trajectory.dynamic_constraints_ignorable = true;
    follow_trajectory.base_time = 0.0;
    follow_trajectory.closed = false;
    std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> follow_trajectory_ptr =
      std::make_shared<traffic_simulator_msgs::msg::PolylineTrajectory>(follow_trajectory);
    api_.requestFollowTrajectory("bob", follow_trajectory_ptr);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<ReverseWalkAtRoadScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
