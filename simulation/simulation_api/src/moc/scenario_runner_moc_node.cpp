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

#include <simulation_api/api/api.hpp>
#include <quaternion_operation/quaternion_operation.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>

// headers in STL
#include <memory>
#include <vector>
#include <string>

// headers in pugixml
#include "pugixml.hpp"

class ScenarioRunnerMoc : public rclcpp::Node
{
public:
  explicit ScenarioRunnerMoc(const rclcpp::NodeOptions & option)
  : Node("scenario_runner", option),
    api_(this, ament_index_cpp::get_package_share_directory(
        "kashiwanoha_map") + "/map/lanelet2_map.osm")
  {
    api_.setVerbose(true);
    api_.initialize(1.0, 0.05);
    pugi::xml_document catalog_xml_doc;
    catalog_xml_doc.load_string(catalog_xml.c_str());
    simulation_api::entity::VehicleParameters params(catalog_xml_doc);
    api_.spawn(false, "ego", params);
    api_.setEntityStatus(
      "ego",
      simulation_api::helper::constractLaneletPose(120545, 0),
      simulation_api::helper::constractActionStatus(10));
    api_.setTargetSpeed("ego", 15, true);
    pugi::xml_document pedestrian_xml_doc;
    pedestrian_xml_doc.load_string(pedestrian_xml.c_str());
    simulation_api::entity::PedestrianParameters pedestrian_params(pedestrian_xml_doc);
    api_.spawn(false, "tom", pedestrian_params);
    api_.setEntityStatus(
      "tom", "ego",
      simulation_api::helper::constractPose(10, 3, 0, 0, 0, 1.57),
      simulation_api::helper::constractActionStatus());
    api_.spawn(
      false, "bob", pedestrian_params,
      simulation_api::helper::constractLaneletPose(34378, 0.0),
      simulation_api::helper::constractActionStatus(1));
    api_.setTargetSpeed("bob", 1, true);
    api_.spawn(
      false, "npc1", params,
      simulation_api::helper::constractLaneletPose(34579, 20.0),
      simulation_api::helper::constractActionStatus(5));
    api_.setTargetSpeed("npc1", 5, true);
    lanechange_excuted_ = false;
    api_.spawn(
      false, "npc2", params,
      simulation_api::helper::constractLaneletPose(34606, 20.0),
      simulation_api::helper::constractActionStatus(5));
    api_.setTargetSpeed("npc2", 0, true);
    api_.requestAcquirePosition(
      "ego",
      simulation_api::helper::constractLaneletPose(34675, 0.0) );
    api_.requestAcquirePosition(
      "npc1",
      simulation_api::helper::constractLaneletPose(34675, 0.0) );
    using namespace std::chrono_literals;
    update_timer_ = this->create_wall_timer(50ms, std::bind(&ScenarioRunnerMoc::update, this));
  }

private:
  void update()
  {
    if (api_.reachPosition(
        "ego",
        simulation_api::helper::constractLaneletPose(34615, 10.0), 5))
    {
      api_.requestAcquirePosition(
        "ego",
        simulation_api::helper::constractLaneletPose(35026, 0.0) );
      api_.setTargetSpeed("npc2", 13, true);
    }
    if (api_.reachPosition(
        "ego",
        simulation_api::helper::constractLaneletPose(34579, 0.0), 5))
    {
      api_.setTargetSpeed("npc2", 3, true);
    }
    if (api_.checkCollision("ego", "npc1")) {
      std::cout << "npc1 collision!" << std::endl;
    }
    if (api_.checkCollision("ego", "npc2")) {
      std::cout << "npc2 collision!" << std::endl;
    }
    api_.updateFrame();
    current_time_ = current_time_ + 0.05;
  }
  bool lanechange_excuted_;
  bool target_speed_setted_;
  bool bob_spawned_;
  double current_time_;
  int port_;
  scenario_simulator::API api_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  std::string catalog_xml =
    R"(<Vehicle name= 'vehicle.volkswagen.t2' vehicleCategory='car'>
            <ParameterDeclarations/>
            <Performance maxSpeed='69.444' maxAcceleration='200' maxDeceleration='10.0'/>
            <BoundingBox>
                <Center x='1.5' y='0.0' z='0.9'/>
                <Dimensions width='2.1' length='4.5' height='1.8'/>
            </BoundingBox>
            <Axles>
                <FrontAxle maxSteering='0.5' wheelDiameter='0.6' trackWidth='1.8' positionX='3.1' positionZ='0.3'/>
                <RearAxle maxSteering='0.0' wheelDiameter='0.6' trackWidth='1.8' positionX='0.0' positionZ='0.3'/>
            </Axles>
            <Properties>
                <Property name='type' value='ego_vehicle'/>
            </Properties>
        </Vehicle>)";

  std::string pedestrian_xml =
    R"(
    <Pedestrian model='bob' mass='0.0' name='Bob' pedestrianCategory='pedestrian'>
            <BoundingBox>
                <Center x='0.0' y='0.0' z='0.5'/>
                <Dimensions width='1.0' length='1.0' height='2.0'/>
            </BoundingBox>
            <Properties/>
        </Pedestrian>)";
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
