// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__CATALOG_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__CATALOG_HPP_

#include <pugixml.hpp>
#include <string>

namespace openscenario_interpreter
{
class Scope;

inline namespace syntax
{
/* ---- Catalog --------------------------------------------------------------
 *
 *  <xsd:complexType name="Catalog">
 *    <xsd:sequence>
 *      <xsd:element name="Vehicle" type="Vehicle" minOccurs="0" maxOccurs="unbounded"/>
 *      <xsd:element name="Controller" type="Controller" minOccurs="0" maxOccurs="unbounded"/>
 *      <xsd:element name="Pedestrian" type="Pedestrian" minOccurs="0" maxOccurs="unbounded"/>
 *      <xsd:element name="MiscObject" type="MiscObject" minOccurs="0" maxOccurs="unbounded"/>
 *      <xsd:element name="Environment" type="Environment" minOccurs="0" maxOccurs="unbounded"/>
 *      <xsd:element name="Maneuver" type="Maneuver" minOccurs="0" maxOccurs="unbounded"/>
 *      <xsd:element name="Trajectory" type="Trajectory" minOccurs="0" maxOccurs="unbounded"/>
 *      <xsd:element name="Route" type="Route" minOccurs="0" maxOccurs="unbounded"/>
 *    </xsd:sequence>
 *    <xsd:attribute name="name" type="String"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Catalog
{
  std::string name;
  explicit Catalog(const pugi::xml_node & node, Scope & scope);
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CATALOG_HPP_
