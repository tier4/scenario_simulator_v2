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
#include <iostream>
// #include <autoware_ad_api_specs/routing.hpp>
// #include <component_interface_utils/rclcpp.hpp>
// #include <map_height_fitter/map_height_fitter.hpp>
// #include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
// #include <component_interface_utils/rclcpp.hpp>
// headers in STL
#include <memory>
#include <string>
#include <vector>
// #include <autoware_ad_api_specs/localization.hpp>


namespace cpp_mock_scenarios
{
class TeleportActionScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit TeleportActionScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "teleport_action", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  void onUpdate() override
  {
    const auto t = api_.getCurrentTime();
    // LCOV_EXCL_START
    if (api_.entityExists("bob") && api_.checkCollision("ego", "bob")) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    /**
     * @note The simulation time is internally managed as a fraction and must exactly equal to x.0
     * in the floating-point literal when the simulation time is an integer multiple of the frame rate frame,
     * so in this case `std::abs(t - 1.0) <= std::numeric Decides that `t == 1.0` is more appropriate than `std::numeric_limits<double>::epsilon();`.
     * @sa https://wandbox.org/permlink/dSNRX7K2bQiqSI7P
     */
    // if (t == 1.0) {
    //   if (t != api_.getCurrentTwist("bob").linear.x) {
    //     stop(cpp_mock_scenarios::Result::FAILURE);
    //   }
    // }
    // if (t >= 6.6) {
    //   if (7.5 >= t) {
    //     if (std::fabs(0.1) <= api_.getCurrentTwist("ego").linear.x) {
    //       stop(cpp_mock_scenarios::Result::FAILURE);
    //     }
    //   } else {
    //     if (0.1 >= api_.getCurrentTwist("ego").linear.x) {
    //       stop(cpp_mock_scenarios::Result::FAILURE);
    //     }
    //   }
    // }
    // LCOV_EXCL_STOP
    if (t >= 30) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
  }

  void onInitialize() override
  {
    new_position_subscriber = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped & message) {
        
        geometry_msgs::msg::PoseStamped goal_msg;
        // auto time = get_clock()->now();
        // goal_msg.header.stamp.sec = 0;
        // goal_msg.header.stamp.nanosec = time.nanoseconds();
        goal_msg.header.frame_id = "map";
        goal_msg.pose = api_.toMapPose(api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34606, 0, 0, 0, 0, 0)));

        api_.respawn("ego", message, goal_msg);
      });
    
    spawnEgoEntity(
      api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34621, 10, 0, 0, 0, 0)),
      {api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34606, 0, 0, 0, 0, 0))},
      getVehicleParameters());

    api_.spawn(
      "bob", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34378, 0.0)),
      getPedestrianParameters());
    api_.setLinearVelocity("bob", 0);
    api_.requestSpeedChange(
      "bob", 1.0, traffic_simulator::speed_change::Transition::LINEAR,
      traffic_simulator::speed_change::Constraint(
        traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, 1.0),
      true);
  }

private:
  bool lanechange_executed_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr new_position_subscriber;
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<cpp_mock_scenarios::TeleportActionScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
