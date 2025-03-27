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
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <vector>

namespace cpp_mock_scenarios
{
class GetDistanceInLaneCoordinateScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit GetDistanceInLaneCoordinateScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "get_longitudinal_distance",
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map", "lanelet2_map.osm",
      __FILE__, false, option)
  {
    start();
  }

private:
  bool requested = false;

  /// @todo it should be separated into the API helper collection for the cpp_mock_scenario
  auto lateralDistance(const std::string & from_entity_name, const std::string & to_entity_name)
    -> std::optional<double>
  {
    const auto & from_entity = api_.getEntity(from_entity_name);
    const auto & to_entity = api_.getEntity(to_entity_name);
    if (from_entity.isInLanelet() && to_entity.isInLanelet()) {
      return traffic_simulator::distance::lateralDistance(
        from_entity.getCanonicalizedLaneletPose().value(),
        to_entity.getCanonicalizedLaneletPose().value(), traffic_simulator::RoutingConfiguration(),
        api_.getHdmapUtils());
    }
    return std::nullopt;
  };

  auto lateralDistance(
    const std::string & from_entity_name, const std::string & to_entity_name,
    const double matching_distance) -> std::optional<double>
  {
    const auto from_entity_lanelet_pose =
      api_.getEntity(from_entity_name).getCanonicalizedLaneletPose(matching_distance);
    const auto to_entity_lanelet_pose =
      api_.getEntity(to_entity_name).getCanonicalizedLaneletPose(matching_distance);
    if (from_entity_lanelet_pose && to_entity_lanelet_pose) {
      return traffic_simulator::distance::lateralDistance(
        from_entity_lanelet_pose.value(), to_entity_lanelet_pose.value(),
        traffic_simulator::RoutingConfiguration(), api_.getHdmapUtils());
    }
    return std::nullopt;
  };

  auto longitudinalDistance(
    const std::string & from_entity_name, const std::string & to_entity_name)
    -> std::optional<double>
  {
    const auto & from_entity = api_.getEntity(from_entity_name);
    const auto & to_entity = api_.getEntity(to_entity_name);
    if (from_entity.isInLanelet() && to_entity.isInLanelet()) {
      return traffic_simulator::distance::longitudinalDistance(
        from_entity.getCanonicalizedLaneletPose().value(),
        to_entity.getCanonicalizedLaneletPose().value(), false, false,
        traffic_simulator::RoutingConfiguration(), api_.getHdmapUtils());
    }
    return std::nullopt;
  }

  void onUpdate() override
  {
    if (api_.getCurrentTime() >= 10.0) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
    const auto distance_to_front = longitudinalDistance("ego", "front");
    const auto distance_to_behind = longitudinalDistance("ego", "behind");
    const auto lateral_none = lateralDistance("ego", "front", 0.1);
    const auto lateral_about_one = lateralDistance("ego", "front", 1.5);
    const auto lateral_to_front = lateralDistance("ego", "front");
    const auto lateral_to_behind = lateralDistance("ego", "behind");
    // LCOV_EXCL_START
    if (lateral_none) {
      return stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (!lateral_about_one) {
      return stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (lateral_about_one && !equals(lateral_about_one.value(), 1.0, 0.001)) {
      return stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (lateral_to_front && !equals(lateral_to_front.value(), 1.0)) {
      return stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (lateral_to_behind && !equals(lateral_to_behind.value(), -1.0)) {
      return stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (!distance_to_front) {
      stop(cpp_mock_scenarios::Result::FAILURE);
      return;
    }
    if (!lateral_to_behind) {
      stop(cpp_mock_scenarios::Result::FAILURE);
      return;
    }
    if (distance_to_front.value() >= 5.1 || 4.9 >= distance_to_front.value()) {
      stop(cpp_mock_scenarios::Result::FAILURE);
      return;
    }
    if (!distance_to_behind) {
      stop(cpp_mock_scenarios::Result::FAILURE);
      return;
    }
    if (distance_to_behind.value() >= -4.9 || -5.1 >= distance_to_behind.value()) {
      stop(cpp_mock_scenarios::Result::FAILURE);
      return;
    }
    // LCOV_EXCL_STOP
  }
  void onInitialize() override
  {
    api_.spawn(
      "ego", traffic_simulator::helper::constructCanonicalizedLaneletPose(34513, 5.0, 0.0),
      getVehicleParameters());
    auto & ego_entity = api_.getEntity("ego");
    ego_entity.setLinearVelocity(10);
    ego_entity.requestSpeedChange(3, true);

    api_.spawn(
      "front", traffic_simulator::helper::constructCanonicalizedLaneletPose(34513, 10.0, 1.0),
      getVehicleParameters());
    auto & front_entity = api_.getEntity("front");
    front_entity.setLinearVelocity(10);
    front_entity.requestSpeedChange(3, true);

    api_.spawn(
      "behind", traffic_simulator::helper::constructCanonicalizedLaneletPose(34513, 0.0, -1.0),
      getVehicleParameters());
    auto & behind_entity = api_.getEntity("behind");
    behind_entity.setLinearVelocity(10);
    behind_entity.requestSpeedChange(3, true);
  }
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component =
    std::make_shared<cpp_mock_scenarios::GetDistanceInLaneCoordinateScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
