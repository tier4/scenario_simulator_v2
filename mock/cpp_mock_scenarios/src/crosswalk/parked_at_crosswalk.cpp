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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cpp_mock_scenarios/catalogs.hpp>
#include <cpp_mock_scenarios/cpp_scenario_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/api/api.hpp>

// headers in STL
#include <memory>
#include <string>
#include <vector>

class ParkedAtCrosswalkScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit ParkedAtCrosswalkScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "parked_at_crosswalk",
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map", "lanelet2_map.osm",
      __FILE__, false, option)
  {
    start();
  }

private:
  void onUpdate() override
  {
    if (api_.getCurrentTime() >= 20) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }

    // LCOV_EXCL_START

    // const auto getOffset = [this](std::string name) {
    //   auto offset =
    //     api_.getEntity(name).getCanonicalizedLaneletPose().value().getLaneletPose().offset;
    //   offset = std::abs(offset);
    //   if (offset < 0.1) {
    //     return 0.0;
    //   } else {
    //     return offset;
    //   }
    // };

    // static double accumulated_offset_normal = 0.0;
    // static double accumulated_offset_gamma = 0.0;

    // accumulated_offset_normal += getOffset("__TEST__bob_normal");
    // accumulated_offset_gamma += getOffset("bob_gamma");

    // if (
    //   api_.getEntity("bob_gamma").getMapPose().position.y > 73745.0 &&
    //   api_.getEntity("__TEST__bob_normal").getMapPose().position.y > 73745.0) {
    //   if (
    //     accumulated_offset_gamma > accumulated_offset_normal &&
    //     std::abs(accumulated_offset_gamma - accumulated_offset_normal) > 5.0) {
    //     stop(cpp_mock_scenarios::Result::SUCCESS);
    //   } else {
    //     stop(cpp_mock_scenarios::Result::FAILURE);
    //   }
    // }
    // LCOV_EXCL_STOP
  }

  void onInitialize() override
  {
    api_.spawn(
      "ego", traffic_simulator::helper::constructCanonicalizedLaneletPose(34981, 0, 0),
      getVehicleParameters());
    auto & ego = api_.getEntity("ego");
    ego.setStatus(
      traffic_simulator::helper::constructCanonicalizedLaneletPose(34981, 0, 0),
      traffic_simulator::helper::constructActionStatus(5.7));
    ego.requestSpeedChange(0, true);
    ego.requestAssignRoute(std::vector<traffic_simulator::CanonicalizedLaneletPose>{
      traffic_simulator::helper::constructCanonicalizedLaneletPose(34675, 0.0, 0.0),
      traffic_simulator::helper::constructCanonicalizedLaneletPose(34690, 0.0, 0.0),
    });

    api_.spawn(
      "bob_gamma", traffic_simulator::helper::constructCanonicalizedLaneletPose(34378, 0.0, 0.0),
      getPedestrianParameters(), traffic_simulator::PedestrianBehavior::contextGamma());
    auto & bob_gamma = api_.getEntity("bob_gamma");
    bob_gamma.requestSpeedChange(
      0.5, traffic_simulator::speed_change::Transition::LINEAR,
      traffic_simulator::speed_change::Constraint(
        traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, 1.0),
      true);
    const geometry_msgs::msg::Pose goal_pose = traffic_simulator::pose::toMapPose(
      traffic_simulator::helper::constructCanonicalizedLaneletPose(34378, 7.5, 0, 0, 0, 0));
    bob_gamma.requestAcquirePosition(goal_pose);

    api_.spawn(
      "__TEST__bob_normal",
      traffic_simulator::helper::constructCanonicalizedLaneletPose(34378, 0.0, 0.0),
      getPedestrianParameters());
    auto & bob_normal = api_.getEntity("__TEST__bob_normal");
    bob_normal.requestSpeedChange(
      0.5, traffic_simulator::speed_change::Transition::LINEAR,
      traffic_simulator::speed_change::Constraint(
        traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, 1.0),
      true);
    bob_normal.requestAcquirePosition(goal_pose);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<ParkedAtCrosswalkScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}