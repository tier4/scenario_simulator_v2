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
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/api/api.hpp>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <vector>

namespace cpp_mock_scenarios
{
class DefineTrafficSourceVehicle : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit DefineTrafficSourceVehicle(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "define_traffic_source_vehicle",
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "private_road_and_walkway_ele_fix/lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  using VehicleBehavior = traffic_simulator::entity::VehicleEntity::BuiltinBehavior;
  using PedestrianBehavior = traffic_simulator::entity::PedestrianEntity::BuiltinBehavior;

  void onUpdate() override
  {
    if (api_.getCurrentTime() >= 3.0) {
      std::vector<std::string> names = api_.getEntityNames();
      names.erase(std::find(names.begin(), names.end(), "ego"));

      if (names.size() != static_cast<std::size_t>(6)) {
        stop(cpp_mock_scenarios::Result::FAILURE);  // LCOV_EXCL_LINE
      }
      for (const auto & name : names) {
        if (const auto entity = api_.getEntity(name)) {
          const bool valid_vehicle_lanelet =
            api_.isInLanelet(name, static_cast<lanelet::Id>(34705), 50.0) ||
            api_.isInLanelet(name, static_cast<lanelet::Id>(34696), 50.0);

          if (!entity->laneMatchingSucceed() || !valid_vehicle_lanelet || !isVehicle(name)) {
            stop(cpp_mock_scenarios::Result::FAILURE);  // LCOV_EXCL_LINE
          }
        }
      }
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
  }

  void onInitialize() override
  {
    api_.addTrafficSource(
      5.0, 2.0, 10.0,
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(3755.0).y(73692.5).z(0.0))
        .orientation(math::geometry::convertEulerAngleToQuaternion(
          geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(0.321802))),
      // clang-format off
      {
        {getVehicleParameters(), VehicleBehavior::defaultBehavior(), "", 0.0},
      }  // clang-format on
      ,
      false, true, true, 0);

    api_.spawn(
      "ego",
      traffic_simulator::helper::constructCanonicalizedLaneletPose(
        34570, 0.0, 0.0, api_.getHdmapUtils()),
      getVehicleParameters());
    api_.setLinearVelocity("ego", 0.0);
    api_.requestSpeedChange("ego", 0.0, true);
  }
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<cpp_mock_scenarios::DefineTrafficSourceVehicle>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
