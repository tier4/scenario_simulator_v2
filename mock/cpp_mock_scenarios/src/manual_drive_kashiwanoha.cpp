// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/api/api.hpp>

// headers in STL
#include <memory>
#include <string>
#include <vector>

// headers in pugixml
#include "pugixml.hpp"

class ScenarioRunnerMoc : public rclcpp::Node
{
public:
  explicit ScenarioRunnerMoc(const rclcpp::NodeOptions & option)
  : Node("scenario_runner", option),
    api_(
      this, __FILE__,
      ament_index_cpp::get_package_share_directory("cargo_delivery") +
        "/maps/kashiwa/lanelet2_map_with_private_road_and_walkway_ele_fix.osm")
  {
    api_.setVerbose(true);
    api_.initialize(1.0, 0.05);
    pugi::xml_document vehicle_catalog_xml_doc;
    Catalog catalog;
    vehicle_catalog_xml_doc.load_string(catalog.vehicle_catalog_xml.c_str());
    api_.spawn(
      false, "ego",
      traffic_simulator::entity::VehicleParameters(vehicle_catalog_xml_doc).toRosMsg());
    api_.setEntityStatus(
      "ego", traffic_simulator::helper::constructLaneletPose(35026, 0, -0.591),
      traffic_simulator::helper::constructActionStatus(0));
    api_.spawn(
      false, "npc",
      traffic_simulator::entity::VehicleParameters(vehicle_catalog_xml_doc).toRosMsg());
    api_.setEntityStatus(
      "npc", traffic_simulator::helper::constructLaneletPose(35026, 10, 0.0),
      traffic_simulator::helper::constructActionStatus(10));
    api_.setTargetSpeed("npc", 5, true);
    /*
    api_.attachLidarSensor(
      traffic_simulator::helper::constructLidarConfiguration(
        traffic_simulator::helper::LidarType::VLP32,
        "ego",
        "points_raw")
    );
    api_.attachDetectionSensor(
      traffic_simulator::helper::constructDetectionSensorConfiguration("ego", "/detection", 0.1));
    */
    using namespace std::chrono_literals;
    update_timer_ = this->create_wall_timer(50ms, std::bind(&ScenarioRunnerMoc::update, this));
  }

private:
  void update()
  {
    api_.updateFrame();
    current_time_ = current_time_ + 0.05;
  }
  bool lanechange_excuted_;
  bool target_speed_setted_;
  bool bob_spawned_;
  double current_time_;
  int port_;
  traffic_simulator::API api_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<ScenarioRunnerMoc>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
