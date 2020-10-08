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

#ifndef SIMULATION_CONTROLLER__ENTITY__PEDESTRIAN_PARAMETER_HPP_
#define SIMULATION_CONTROLLER__ENTITY__PEDESTRIAN_PARAMETER_HPP_

// headers in pugixml
#include <pugixml.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <simulation_api/entity/vehicle_parameter.hpp>

#include <string>
#include <sstream>

namespace simulation_api
{
namespace entity
{
struct PedestrianParameters
{
  explicit PedestrianParameters(const pugi::xml_node & xml)
  : name(xml.child("Pedestrian").attribute("name").as_string()),
    pedestrian_categoly(xml.child("Pedestrian").attribute("pedestrianCategory").as_string()),
    bounding_box(xml.child("Pedestrian"))
  {}
  PedestrianParameters(
    std::string name,
    std::string pedestrian_categoly, BoundingBox bounding_box)
  : name(name),
    pedestrian_categoly(pedestrian_categoly),
    bounding_box(bounding_box)
  {}
  const std::string name;
  const std::string pedestrian_categoly;
  const BoundingBox bounding_box;

  std::string toXml()
  {
    using boost::property_tree::ptree;
    ptree pt;
    ptree & pedestrian_tree = pt.add("Pedestrian", "");
    pedestrian_tree.put("<xmlattr>.name", name);
    pedestrian_tree.put("<xmlattr>.PedestrianCategory", pedestrian_categoly);
    ptree & center_tree = pedestrian_tree.add("BoundingBox.Center", "");
    center_tree.put("<xmlattr>.x", bounding_box.center.x);
    center_tree.put("<xmlattr>.y", bounding_box.center.y);
    center_tree.put("<xmlattr>.z", bounding_box.center.z);
    ptree & dimensions_tree = pedestrian_tree.add("BoundingBox.Dimensions", "");
    dimensions_tree.put("<xmlattr>.width", bounding_box.dimensions.width);
    dimensions_tree.put("<xmlattr>.length", bounding_box.dimensions.length);
    dimensions_tree.put("<xmlattr>.height", bounding_box.dimensions.height);
    std::stringstream ss;
    boost::property_tree::write_xml(ss, pt);
    return ss.str();
  }
};
}  // namespace entity
}  // namespace simulation_api

#endif  // SIMULATION_CONTROLLER__ENTITY__PEDESTRIAN_PARAMETER_HPP_
