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
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/api/api.hpp>
#include <traffic_simulator/metrics/metrics.hpp>

// headers in STL
#include <memory>
#include <string>
#include <utility>
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
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map/lanelet2_map.osm")
  {
    api_.setVerbose(true);
    api_.initialize(1.0, 0.05);
    pugi::xml_document catalog_xml_doc;
    catalog_xml_doc.load_string(catalog_xml.c_str());
    auto vehicle_params = traffic_simulator::entity::VehicleParameters(catalog_xml_doc).toRosMsg();
    vehicle_params.name = "ego";
    api_.spawn(false, "ego", vehicle_params);
    api_.setEntityStatus(
      "ego", traffic_simulator::helper::constructLaneletPose(120545, 0),
      traffic_simulator::helper::constructActionStatus(10));
    api_.setTargetSpeed("ego", 15, true);
    pugi::xml_document pedestrian_xml_doc;
    pedestrian_xml_doc.load_string(pedestrian_xml.c_str());
    const auto pedestrian_params =
      traffic_simulator::entity::PedestrianParameters(pedestrian_xml_doc).toRosMsg();
    api_.spawn(false, "tom", pedestrian_params);
    api_.setEntityStatus(
      "tom", "ego", traffic_simulator::helper::constructPose(10, 3, 0, 0, 0, -1.57),
      traffic_simulator::helper::constructActionStatus());
    api_.requestWalkStraight("tom");
    api_.setTargetSpeed("tom", 3, true);
    api_.spawn(
      false, "bob", pedestrian_params, traffic_simulator::helper::constructLaneletPose(34378, 0.0),
      traffic_simulator::helper::constructActionStatus(1));
    api_.setTargetSpeed("bob", 1, true);
    vehicle_params.name = "npc1";
    api_.spawn(
      false, "npc1", vehicle_params, traffic_simulator::helper::constructLaneletPose(34579, 20.0),
      traffic_simulator::helper::constructActionStatus(5));
    api_.setTargetSpeed("npc1", 5, true);
    lanechange_excuted_ = false;
    vehicle_params.name = "npc2";
    api_.spawn(
      false, "npc2", vehicle_params, traffic_simulator::helper::constructLaneletPose(34606, 20.0),
      traffic_simulator::helper::constructActionStatus(5));
    api_.setTargetSpeed("npc2", 0, true);
    api_.requestAssignRoute(
      "ego", std::vector<openscenario_msgs::msg::LaneletPose>{
               traffic_simulator::helper::constructLaneletPose(34675, 0.0),
               traffic_simulator::helper::constructLaneletPose(34690, 0.0)});
    api_.requestAcquirePosition(
      "npc1", traffic_simulator::helper::constructLaneletPose(34675, 0.0));
    api_.spawn(false, "npc3", vehicle_params);
    api_.setEntityStatus(
      "npc3", traffic_simulator::helper::constructLaneletPose(34468, 0),
      traffic_simulator::helper::constructActionStatus(10));
    /*
    api_.addMetric<metrics::TraveledDistanceMetric>("ego_traveled_distance", "ego");
    api_.addMetric<metrics::MomentaryStopMetric>(
      "ego_momentary_stop0", "ego",
      -10, 10, 34805, metrics::MomentaryStopMetric::StopTargetLaneletType::STOP_LINE,
      30, 1, 0.05);
    api_.addMetric<metrics::MomentaryStopMetric>(
      "ego_momentary_stop1", "ego",
      -10, 10, 120635, metrics::MomentaryStopMetric::StopTargetLaneletType::STOP_LINE,
      30, 1, 0.05);
    api_.addMetric<metrics::MomentaryStopMetric>(
      "ego_momentary_stop_crosswalk", "ego",
      -10, 10, 34378, metrics::MomentaryStopMetric::StopTargetLaneletType::CROSSWALK,
      30, 1, 0.05);
    */
    std::vector<std::pair<double, traffic_simulator::TrafficLightColor>> phase;
    phase = {
      {10, traffic_simulator::TrafficLightColor::GREEN},
      {10, traffic_simulator::TrafficLightColor::YELLOW},
      {10, traffic_simulator::TrafficLightColor::RED}};
    api_.setTrafficLightColorPhase(34802, phase);
    using namespace std::chrono_literals;
    update_timer_ = this->create_wall_timer(50ms, std::bind(&ScenarioRunnerMoc::update, this));
  }

private:
  void update()
  {
    if (api_.getCurrentTime() >= 4 && api_.entityExists("tom")) {
      api_.despawn("tom");
    }
    /*
    if (api_.getLinearJerk("ego")) {
      std::cout << "ego linear jerk :" << api_.getLinearJerk("ego").get() << std::endl;
    }
    */
    if (api_.reachPosition(
          "ego", traffic_simulator::helper::constructLaneletPose(34615, 10.0), 5)) {
      api_.requestAcquirePosition(
        "ego", traffic_simulator::helper::constructLaneletPose(35026, 0.0));
      if (api_.entityExists("npc2")) {
        api_.setTargetSpeed("npc2", 13, true);
      }
    }
    if (api_.reachPosition("ego", traffic_simulator::helper::constructLaneletPose(34579, 0.0), 5)) {
      api_.requestAcquirePosition(
        "ego", traffic_simulator::helper::constructLaneletPose(34675, 0.0));
      if (api_.entityExists("npc2")) {
        api_.setTargetSpeed("npc2", 3, true);
      }
    }
    if (api_.reachPosition(
          "npc2", traffic_simulator::helper::constructLaneletPose(34513, 0.0), 5)) {
      api_.requestAcquirePosition(
        "npc2", traffic_simulator::helper::constructLaneletPose(34630, 0.0));
      api_.setTargetSpeed("npc2", 13, true);
    }
    /*
    if (api_.checkCollision("ego", "npc1")) {
      std::cout << "npc1 collision!" << std::endl;
    }
    if (api_.checkCollision("ego", "npc2")) {
      std::cout << "npc2 collision!" << std::endl;
    }
    */
    if (current_time_ > 10.0 && api_.entityExists("bob")) {
      api_.despawn("bob");
    }
    api_.updateFrame();
    current_time_ = current_time_ + 0.05;
  }
  bool lanechange_excuted_;
  bool target_speed_setted_;
  double current_time_;
  int port_;
  traffic_simulator::API api_;
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
    R"(<Pedestrian model='bob' mass='0.0' name='Bob' pedestrianCategory='pedestrian'>
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
  const auto component = std::make_shared<ScenarioRunnerMoc>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
