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
#include <vector>

namespace cpp_mock_scenarios
{
class RespawnEgoScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit RespawnEgoScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "respawn_ego", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "lanelet2_map.osm", __FILE__, false, option),
    goal_pose{traffic_simulator::helper::constructLaneletPose(34606, 0, 0)},
    new_position_subscriber{create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped & message) {
        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.frame_id = "map";
        goal_msg.pose = static_cast<geometry_msgs::msg::Pose>(goal_pose);
        api_.respawn("ego", message, goal_msg);
      })},
    has_done_respawn{false}
  {
    start();
  }

private:
  void onUpdate() override
  {
    if (api_.getCurrentTime() >= 60) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }

    if (api_.isInLanelet("ego", 34564, 0.1)) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }

    if (api_.getCurrentTime() >= 10 && !has_done_respawn) {
      geometry_msgs::msg::PoseWithCovarianceStamped ego_pose;
      ego_pose.header.frame_id = "map";
      ego_pose.pose.pose = static_cast<geometry_msgs::msg::Pose>(
        traffic_simulator::helper::constructCanonicalizedLaneletPose(
          34576, 10.0, 0.0, api_.getHdmapUtils()));

      geometry_msgs::msg::PoseStamped goal_pose;
      goal_pose.header.frame_id = "map";
      goal_pose.pose = static_cast<geometry_msgs::msg::Pose>(
        traffic_simulator::helper::constructCanonicalizedLaneletPose(
          34564, 10.0, 0.0, api_.getHdmapUtils()));
      api_.respawn("ego", ego_pose, goal_pose);

      has_done_respawn = true;
    }
  }

  void onInitialize() override
  {
    spawnEgoEntity(
      traffic_simulator::helper::constructLaneletPose(34621, 10.0, 0.0), {goal_pose},
      getVehicleParameters());
  }

private:
  const traffic_simulator_msgs::msg::LaneletPose goal_pose;

  const rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    new_position_subscriber;

  bool has_done_respawn;
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<cpp_mock_scenarios::RespawnEgoScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
