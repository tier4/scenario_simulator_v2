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

//headers in Google Test
#include <gtest/gtest.h>

//headers in ROS
#include <ros/ros.h>
#include <ros/package.h>

// headers in this package
#include <simulation_controller/entity/entity_manager.hpp>

// headers in pugixml
#include "pugixml.hpp"

std::string catalog_xml =
  "<Vehicle name= 'vehicle.volkswagen.t2' vehicleCategory='car'>\
    <ParameterDeclarations/>\
    <Performance maxSpeed='69.444' maxAcceleration='200' maxDeceleration='10.0'/>\
    <BoundingBox>\
        <Center x='1.5' y='0.0' z='0.9'/>\
        <Dimensions width='2.1' length='4.5' height='1.8'/>\
    </BoundingBox>\
    <Axles>\
        <FrontAxle maxSteering='0.5' wheelDiameter='0.6' trackWidth='1.8' positionX='3.1' positionZ='0.3'/>\
        <RearAxle maxSteering='0.0' wheelDiameter='0.6' trackWidth='1.8' positionX='0.0' positionZ='0.3'/>\
    </Axles>\
    <Properties>\
        <Property name='type' value='ego_vehicle'/>\
    </Properties>\
</Vehicle>";

simulation_controller::entity::EntityStatus getInitialStatus()
{
  geometry_msgs::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  geometry_msgs::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;
  geometry_msgs::Accel accel;
  accel.linear.x = 0.0;
  accel.linear.y = 0.0;
  accel.linear.z = 0.0;
  accel.angular.x = 0.0;
  accel.angular.y = 0.0;
  accel.angular.z = 0.0;
  simulation_controller::entity::EntityStatus ret(0.0, pose, twist, accel);
}

TEST(TestSuite, testCase1)
{
  pugi::xml_document volkswagen;
  volkswagen.load_string(catalog_xml.c_str());
  //pugi::xml_node volkswagen = loadXml("vehicle.volkswagen.t2");
  //pugi::xml_node tesla = loadXml("vehicle.tesla.model3");
  simulation_controller::entity::EntityManager manager(true);
  simulation_controller::entity::EgoEntity ego("ego", getInitialStatus(), volkswagen);
  EXPECT_DOUBLE_EQ(ego.parameters.performance.max_speed, 69.444000000000003);
  EXPECT_DOUBLE_EQ(ego.parameters.axles.front_axle.max_steering, 0.5);
  EXPECT_EQ(manager.spawnEntity(ego), true);
  //EXPECT_EQ(manager.getEntity(ego)->name, ego.name);
  EXPECT_EQ(ego.type, "vehicle.volkswagen.t2");
  simulation_controller::entity::VehicleEntity npc("npc", getInitialStatus(), volkswagen);
  EXPECT_EQ(manager.spawnEntity(npc), true);
  //EXPECT_EQ(manager.getEntity(npc)->name, npc.name);
  EXPECT_EQ(manager.despawnEntity(npc.name), true);
  EXPECT_EQ(manager.despawnEntity(npc.name), false);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
