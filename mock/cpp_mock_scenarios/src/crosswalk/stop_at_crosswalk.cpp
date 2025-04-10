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
#include <vector>

namespace cpp_mock_scenarios
{
class StopAtCrosswalkScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit StopAtCrosswalkScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "stop_at_crosswalk", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "lanelet2_map.osm", __FILE__, false, option, {traffic_simulator::EntityType::PEDESTRIAN})
  {
    start();
  }

private:
  void onUpdate() override
  {
    const auto t = api_.getCurrentTime();
    // LCOV_EXCL_START
    if (api_.isEntityExist("bob") && api_.checkCollision("ego", "bob")) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    /**
     * @note The simulation time is internally managed as a fraction and must exactly equal to x.0
     * in the floating-point literal when the simulation time is an integer multiple of the frame rate frame,
     * so in this case `std::abs(t - 1.0) <= std::numeric Decides that `t == 1.0` is more appropriate than `std::numeric_limits<double>::epsilon();`.
     * @sa https://wandbox.org/permlink/dSNRX7K2bQiqSI7P
     */
    if (t == 1.0) {
      if (t != api_.getEntity("bob").getCurrentTwist().linear.x) {
        stop(cpp_mock_scenarios::Result::FAILURE);
      }
    }
    const auto ego_linear_velocity = api_.getEntity("ego").getCurrentTwist().linear.x;
    if (t >= 6.6) {
      if (7.5 >= t) {
        if (std::fabs(0.1) <= ego_linear_velocity) {
          stop(cpp_mock_scenarios::Result::FAILURE);
        }
      } else {
        if (0.1 >= ego_linear_velocity) {
          stop(cpp_mock_scenarios::Result::FAILURE);
        }
      }
    }
    // LCOV_EXCL_STOP
    if (t >= 10) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
  }

  void onInitialize() override
  {
    api_.spawn(
      "ego", traffic_simulator::helper::constructCanonicalizedLaneletPose(120545, 0.0, 0.0),
      getVehicleParameters());
    auto & ego_entity = api_.getEntity("ego");
    ego_entity.setLinearVelocity(10);
    ego_entity.requestSpeedChange(8, true);
    ego_entity.requestAssignRoute(std::vector<traffic_simulator::CanonicalizedLaneletPose>{
      traffic_simulator::helper::constructCanonicalizedLaneletPose(34675, 0.0, 0.0),
      traffic_simulator::helper::constructCanonicalizedLaneletPose(34690, 0.0, 0.0)});

    api_.spawn(
      "bob", traffic_simulator::helper::constructCanonicalizedLaneletPose(34378, 0.0, 0.0),
      getPedestrianParameters());
    auto & bob_entity = api_.getEntity("bob");
    bob_entity.setLinearVelocity(0);
    bob_entity.requestSpeedChange(
      1.0, traffic_simulator::speed_change::Transition::LINEAR,
      traffic_simulator::speed_change::Constraint(
        traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, 1.0),
      true);
  }

private:
  bool lanechange_executed_;
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<cpp_mock_scenarios::StopAtCrosswalkScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
