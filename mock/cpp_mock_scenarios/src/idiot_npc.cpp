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
#include <openscenario_msgs/msg/driver_model.hpp>
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
  : Node("scenario_runner", option), api_(this, configure())
  {
    api_.initialize(1.0, 0.05);
    pugi::xml_document vehicle_catalog_xml_doc;
    Catalog catalog;
    vehicle_catalog_xml_doc.load_string(catalog.vehicle_catalog_xml.c_str());
    api_.spawn(
      false, "idiot",
      traffic_simulator::entity::VehicleParameters(vehicle_catalog_xml_doc).toRosMsg());
    api_.setEntityStatus(
      "idiot", traffic_simulator::helper::constructLaneletPose(34741, 0, 0),
      traffic_simulator::helper::constructActionStatus(0));
    api_.setTargetSpeed("idiot", 15, true);
    openscenario_msgs::msg::DriverModel driver_model;
    driver_model.see_around = false;
    api_.setDriverModel("idiot", driver_model);
    api_.spawn(
      false, "npc",
      traffic_simulator::entity::VehicleParameters(vehicle_catalog_xml_doc).toRosMsg());
    api_.setEntityStatus(
      "npc", traffic_simulator::helper::constructLaneletPose(34741, 10, 0),
      traffic_simulator::helper::constructActionStatus(0));
    api_.setTargetSpeed("npc", 5, true);
    using namespace std::chrono_literals;
    update_timer_ = this->create_wall_timer(50ms, std::bind(&ScenarioRunnerMoc::update, this));
  }

private:
  void update()
  {
    if (!api_.checkCollision("idiot", "npc")) {
      api_.updateFrame();
      current_time_ = current_time_ + 0.05;
    } else {
      if (current_time_ <= 3.0) {
        rclcpp::shutdown();
        std::exit(-1);
      }
      std::cerr << "ERROR" << std::endl;
      trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
      update_timer_->cancel();
    }
  }

  static auto configure() -> traffic_simulator::Configuration
  {
    auto configuration = traffic_simulator::Configuration(
      ament_index_cpp::get_package_share_directory("cargo_delivery") + "/maps/kashiwa");
    {
      configuration.lanelet2_map_file = "lanelet2_map_with_private_road_and_walkway_ele_fix.osm";
      configuration.scenario_path = __FILE__;
      configuration.verbose = true;
    }

    return configuration;
  }

  bool lanechange_executed_;
  bool target_speed_set_;
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
