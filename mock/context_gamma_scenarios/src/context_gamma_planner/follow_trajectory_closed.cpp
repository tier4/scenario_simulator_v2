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
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/api/api.hpp>

// headers in STL
#include <memory>
#include <string>
#include <vector>

class FollowTrajectoryClosedScenario : public context_gamma_scenarios::ContextGammaScenarioNode
{
public:
  explicit FollowTrajectoryClosedScenario(const rclcpp::NodeOptions & option)
  : context_gamma_scenarios::ContextGammaScenarioNode(
      "follow_trajectory_closed",
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map", "lanelet2_map.osm",
      __FILE__, false, option)
  {
    start();
  }

private:
  traffic_simulator_msgs::msg::PolylineTrajectory follow_trajectory;
  int vertex_num = 0;
  int loop_count = 0;
  Eigen::Vector3d toEigenVector3d(const geometry_msgs::msg::Point & pos)
  {
    return Eigen::Vector3d(pos.x, pos.y, pos.z);
  }
  void onUpdate() override
  {
    const auto t = api_.getCurrentTime();
    const double distance_threshold = 5.0;
    const auto ego_pos = toEigenVector3d(api_.getEntityStatus("ego").getMapPose().position);
    const auto reference_pos =
      toEigenVector3d(follow_trajectory.shape.vertices.at(vertex_num).position.position);
    if ((ego_pos - reference_pos).norm() < distance_threshold) {
      if (vertex_num == static_cast<int>(follow_trajectory.shape.vertices.size()) - 1) {
        loop_count++;
        vertex_num = 0;
        RCLCPP_INFO_STREAM(get_logger(), "loop count: " << loop_count);
        if (loop_count == 2) {
          stop(context_gamma_scenarios::Result::SUCCESS);
        }
      } else {
        vertex_num++;
      }
      RCLCPP_INFO_STREAM(get_logger(), "passing vertex number: " << vertex_num);
    }
    // LCOV_EXCL_STOP
    if (t >= 100) {
      stop(context_gamma_scenarios::Result::FAILURE);
    }
  }

  void onInitialize() override
  {
    //Vehicle setting
    api_.spawn(
      "ego",
      traffic_simulator::helper::constructCanonicalizedLaneletPose(
        34579, 0, 0, api_.getHdmapUtils()),
      getVehicleParameters(), traffic_simulator::VehicleBehavior::contextGamma());
    api_.setEntityStatus(
      "ego",
      (traffic_simulator::helper::constructCanonicalizedLaneletPose(
        34579, 0, 0, api_.getHdmapUtils())),
      traffic_simulator::helper::constructActionStatus(10));
    api_.requestSpeedChange("ego", 7, true);

    auto toVertex = [](double time, const geometry_msgs::msg::Pose & pose) {
      traffic_simulator_msgs::msg::Vertex vertex;
      vertex.position = pose;
      vertex.time = time;
      return vertex;
    };
    follow_trajectory.shape.vertices.push_back(toVertex(
      0.0, traffic_simulator::pose::toMapPose(
             traffic_simulator::helper::constructCanonicalizedLaneletPose(
               34579, 0.0, 0, 0, 0, 0, api_.getHdmapUtils()))));
    follow_trajectory.shape.vertices.push_back(toVertex(
      2.0, traffic_simulator::pose::toMapPose(
             traffic_simulator::helper::constructCanonicalizedLaneletPose(
               34579, 10.0, 0, 0, 0, 0, api_.getHdmapUtils()))));
    follow_trajectory.shape.vertices.push_back(toVertex(
      4.0, traffic_simulator::pose::toMapPose(
             traffic_simulator::helper::constructCanonicalizedLaneletPose(
               34600, 30.0, 0, 0, 0, 0, api_.getHdmapUtils()))));
    follow_trajectory.shape.vertices.push_back(toVertex(
      6.0, traffic_simulator::pose::toMapPose(
             traffic_simulator::helper::constructCanonicalizedLaneletPose(
               34600, 20.0, 0, 0, 0, 0, api_.getHdmapUtils()))));
    follow_trajectory.shape.vertices.push_back(toVertex(
      8.0, traffic_simulator::pose::toMapPose(
             traffic_simulator::helper::constructCanonicalizedLaneletPose(
               34579, 40.0, 0, 0, 0, 0, api_.getHdmapUtils()))));
    follow_trajectory.shape.vertices.push_back(toVertex(
      10.0, traffic_simulator::pose::toMapPose(
              traffic_simulator::helper::constructCanonicalizedLaneletPose(
                34579, 50.0, 0, 0, 0, 0, api_.getHdmapUtils()))));

    follow_trajectory.shape.vertices.push_back(toVertex(
      12.0, traffic_simulator::pose::toMapPose(
              traffic_simulator::helper::constructCanonicalizedLaneletPose(
                34600, 0.0, 0, 0, 0, 0, api_.getHdmapUtils()))));
    follow_trajectory.shape.vertices.push_back(toVertex(
      14.0, traffic_simulator::pose::toMapPose(
              traffic_simulator::helper::constructCanonicalizedLaneletPose(
                34600, 10.0, 0, 0, 0, 0, api_.getHdmapUtils()))));
    follow_trajectory.shape.vertices.push_back(toVertex(
      16.0, traffic_simulator::pose::toMapPose(
              traffic_simulator::helper::constructCanonicalizedLaneletPose(
                34579, 30.0, 0, 0, 0, 0, api_.getHdmapUtils()))));
    follow_trajectory.shape.vertices.push_back(toVertex(
      18.0, traffic_simulator::pose::toMapPose(
              traffic_simulator::helper::constructCanonicalizedLaneletPose(
                34579, 20.0, 0, 0, 0, 0, api_.getHdmapUtils()))));
    follow_trajectory.shape.vertices.push_back(toVertex(
      20.0, traffic_simulator::pose::toMapPose(
              traffic_simulator::helper::constructCanonicalizedLaneletPose(
                34600, 40.0, 0, 0, 0, 0, api_.getHdmapUtils()))));
    follow_trajectory.shape.vertices.push_back(toVertex(
      22.0, traffic_simulator::pose::toMapPose(
              traffic_simulator::helper::constructCanonicalizedLaneletPose(
                34600, 50.0, 0, 0, 0, 0, api_.getHdmapUtils()))));
    follow_trajectory.shape.vertices.push_back(toVertex(
      24.0, traffic_simulator::pose::toMapPose(
              traffic_simulator::helper::constructCanonicalizedLaneletPose(
                34579, 0.0, 0, 0, 0, 0, api_.getHdmapUtils()))));

    follow_trajectory.initial_distance_offset = 0.0;
    follow_trajectory.dynamic_constraints_ignorable = true;
    follow_trajectory.base_time = 5.0;
    follow_trajectory.closed = true;
    std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> follow_trajectory_ptr =
      std::make_shared<traffic_simulator_msgs::msg::PolylineTrajectory>(follow_trajectory);
    api_.requestFollowTrajectory("ego", follow_trajectory_ptr);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<FollowTrajectoryClosedScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
