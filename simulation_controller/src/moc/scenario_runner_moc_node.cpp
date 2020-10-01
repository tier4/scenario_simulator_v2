// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include <simulation_controller/api/api.hpp>
#include <quaternion_operation/quaternion_operation.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <xmlrpcpp/XmlRpcClient.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>

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
  : Node("scenario_runner", option), api_(this)
  {
    api_.simulation->initialize(1.0, 0.02);
    pugi::xml_document catalog_xml_doc;
    catalog_xml_doc.load_string(catalog_xml.c_str());
    simulation_controller::entity::VehicleParameters params(catalog_xml_doc);
    api_.entity->spawn(true, "ego", params);
    api_.entity->setEntityStatus("ego", getEgoInitialStatus());
    /*
    lanechange_excuted_ = false;
    api_.entity->spawn(false, "npc1", params, getNpcInitialStatus());
    api_.entity->requestAcquirePosition("npc1", 180, 0, 0);
    pugi::xml_document pedestrian_xml_doc;
    pedestrian_xml_doc.load_string(pedestrian_xml.c_str());
    simulation_controller::entity::PedestrianParameters pedestrian_params(pedestrian_xml_doc);
    api_.entity->spawn(false, "bob", pedestrian_params);
    api_.entity->setVerbose(false);
    current_time_ = 0.0;
    target_speed_setted_ = false;
    lanechange_excuted_ = false;
    bob_spawned_ = false;
    api_.entity->setTargetSpeed("npc1", 10, true);
    update_timer_ = this->create_wall_timer(20ms, std::bind(&ScenarioRunnerMoc::update, this));
    */
    using namespace std::chrono_literals;
    update_timer_ = this->create_wall_timer(20ms, std::bind(&ScenarioRunnerMoc::update, this));
  }

private:
  void update()
  {
    /*
    auto stand_still_duration = api_.entity->getStandStillDuration("ego");
    if (stand_still_duration) {
      if (stand_still_duration.get() > 0.1) {
        std::cout << "ego is stopping " << stand_still_duration.get() << " seconds" << std::endl;
      }
    }
    XmlRpc::XmlRpcValue result;
    if (api_.entity->reachPosition("ego", 180, 0, 0, 10)) {
      if (!bob_spawned_) {
        bob_spawned_ = true;
        api_.entity->setEntityStatus("bob", getBobInitialStatus());
        api_.entity->setTargetSpeed("bob", 0.5, true);
      }
    }
    auto dist = api_.entity->getLongitudinalDistance("ego", "npc1");
    if (dist) {
      if (dist.get() < 25 && api_.entity->isInLanelet("ego", 178)) {
        api_.entity->requestLaneChange("ego", 179);
        lanechange_excuted_ = true;
      }
    }
    if (api_.entity->isInLanelet("ego", 179) && lanechange_excuted_ && !target_speed_setted_) {
      api_.entity->setTargetSpeed("ego", 25, true);
      target_speed_setted_ = true;
    }
    auto time_headway = api_.entity->getTimeHeadway("ego", "npc1");
    if (time_headway) {
      if (time_headway.get() > 1 && api_.entity->isInLanelet("ego", 179)) {
        api_.entity->setVerbose(true);
        api_.entity->setTargetSpeed("npc1", 20, true);
        api_.entity->requestLaneChange("ego", simulation_controller::entity::Direction::LEFT);
      }
    }
    */
    RCLCPP_INFO(get_logger(),"current time : " + std::to_string(current_time_) + " [sec]");
    api_.simulation->updateFrame();
    current_time_ = current_time_ + 0.02;
  }
  bool lanechange_excuted_;
  bool target_speed_setted_;
  bool bob_spawned_;
  double current_time_;
  int port_;
  scenario_simulator::API api_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  simulation_controller::entity::EntityStatus getEgoInitialStatus()
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    geometry_msgs::msg::Accel accel;
    accel.linear.x = 0.0;
    accel.linear.y = 0.0;
    accel.linear.z = 0.0;
    accel.angular.x = 0.0;
    accel.angular.y = 0.0;
    accel.angular.z = 0.0;
    geometry_msgs::msg::Vector3 rpy;
    rpy.x = 0.0;
    rpy.y = 0.0;
    rpy.z = 0.0;
    simulation_controller::entity::EntityStatus ret(
      api_.simulation->getCurrentTime(), 17, 0.0, 0.0, rpy, twist, accel);
    return ret;
  }

  simulation_controller::entity::EntityStatus getNpcInitialStatus()
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 5.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    geometry_msgs::msg::Accel accel;
    accel.linear.x = 0.0;
    accel.linear.y = 0.0;
    accel.linear.z = 0.0;
    accel.angular.x = 0.0;
    accel.angular.y = 0.0;
    accel.angular.z = 0.0;
    geometry_msgs::msg::Vector3 rpy;
    rpy.x = 0.0;
    rpy.y = 0.0;
    rpy.z = 0.0;
    simulation_controller::entity::EntityStatus ret(
      api_.simulation->getCurrentTime(), 178, 20.0, 0.0, rpy, twist, accel);
    return ret;
  }

  simulation_controller::entity::EntityStatus getBobInitialStatus()
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    geometry_msgs::msg::Accel accel;
    accel.linear.x = 0.0;
    accel.linear.y = 0.0;
    accel.linear.z = 0.0;
    accel.angular.x = 0.0;
    accel.angular.y = 0.0;
    accel.angular.z = 0.0;
    geometry_msgs::msg::Vector3 rpy;
    rpy.x = 0.0;
    rpy.y = 0.0;
    rpy.z = 0.0;
    simulation_controller::entity::EntityStatus ret(
      api_.simulation->getCurrentTime(), 879, 0.0, 0.0, rpy, twist, accel);
    return ret;
  }

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
