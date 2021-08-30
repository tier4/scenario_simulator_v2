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

#ifndef CPP_MOCK_SCENARIOS__CATALOGS_HPP_
#define CPP_MOCK_SCENARIOS__CATALOGS_HPP_

#include <openscenario_msgs/msg/misc_object_parameters.hpp>
#include <string>
#include <traffic_simulator/entity/pedestrian_parameter.hpp>
#include <traffic_simulator/entity/vehicle_parameter.hpp>

struct Catalog
{
  std::string vehicle_catalog_xml =
    R"(<Vehicle name= 'vehicle.volkswagen.t2' vehicleCategory='car'>
            <ParameterDeclarations/>
            <Performance maxSpeed='69.444' maxAcceleration='200' maxDeceleration='10.0'/>
            <BoundingBox>
                <Center x='1.5' y='0.0' z='0.9'/>
                <Dimensions width='1.245' length='2.178' height='1.87'/>
            </BoundingBox>
            <Axles>
                <FrontAxle maxSteering='0.5' wheelDiameter='0.6' trackWidth='1.8' positionX='3.1' positionZ='0.3'/>
                <RearAxle maxSteering='0.0' wheelDiameter='0.6' trackWidth='1.8' positionX='0.0' positionZ='0.3'/>
            </Axles>
            <Properties>
                <Property name='type' value='ego_vehicle'/>
            </Properties>
        </Vehicle>)";

  std::string pedestrian_catalog_xml =
    R"(
    <Pedestrian model='bob' mass='0.0' name='Bob' pedestrianCategory='pedestrian'>
            <BoundingBox>
                <Center x='0.0' y='0.0' z='1.0'/>
                <Dimensions width='0.8' length='0.8' height='2.0'/>
            </BoundingBox>
            <Properties/>
        </Pedestrian>)";
};

auto getVehicleParameters() -> openscenario_msgs::msg::VehicleParameters
{
  pugi::xml_document catalog_xml_doc;
  Catalog catalog;
  catalog_xml_doc.load_string(catalog.vehicle_catalog_xml.c_str());
  return traffic_simulator::entity::VehicleParameters(catalog_xml_doc).toRosMsg();
}

auto getPedestrianParameters() -> openscenario_msgs::msg::PedestrianParameters
{
  pugi::xml_document catalog_xml_doc;
  Catalog catalog;
  catalog_xml_doc.load_string(catalog.pedestrian_catalog_xml.c_str());
  return traffic_simulator::entity::PedestrianParameters(catalog_xml_doc).toRosMsg();
}

auto getMiscObjectParameters() -> openscenario_msgs::msg::MiscObjectParameters
{
  openscenario_msgs::msg::MiscObjectParameters misc_object_param;
  misc_object_param.bounding_box.dimensions.x = 1.0;
  misc_object_param.bounding_box.dimensions.y = 1.0;
  misc_object_param.bounding_box.dimensions.z = 1.0;
  misc_object_param.misc_object_category = "obstacle";
  misc_object_param.name = "obstacle";
  return misc_object_param;
}

#endif  // CPP_MOCK_SCENARIOS__CATALOGS_HPP_
