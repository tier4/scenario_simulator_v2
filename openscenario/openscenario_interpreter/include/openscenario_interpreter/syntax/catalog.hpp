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

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/controller.hpp>
#include <openscenario_interpreter/syntax/maneuver.hpp>
#include <openscenario_interpreter/syntax/misc_object.hpp>
#include <openscenario_interpreter/syntax/pedestrian.hpp>
#include <openscenario_interpreter/syntax/route.hpp>
#include <openscenario_interpreter/syntax/vehicle.hpp>
#include "openscenario_interpreter/reader/attribute.hpp"
#include "openscenario_interpreter/reader/element.hpp"
#include "scenario_simulator_exception/exception.hpp"

namespace openscenario_interpreter
{
inline namespace syntax
{
struct CatalogDeclaration
{
  std::string name;
};

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

  template <typename Node>
  explicit Catalog(const Node & node, Scope & scope)
  : name(readAttribute<std::string>("name", node, scope))
  {
    bool already_found = false;

#define FIND_CATEGORY_ELEMENT(TYPE)                                                   \
  do {                                                                                \
    auto elements = readElementsAsElement<TYPE, 0>(#TYPE, node, scope);               \
    if (not elements.empty()) {                                                       \
      if (already_found) {                                                            \
        THROW_SYNTAX_ERROR("Only one type can be defined in a single category file"); \
      }                                                                               \
      already_found = true;                                                           \
      for (Element & element : elements) {                                            \
        scope.insert(element.template as<TYPE>().name, element);                      \
      }                                                                               \
    }                                                                                 \
  } while (0)

    FIND_CATEGORY_ELEMENT(Vehicle);
    FIND_CATEGORY_ELEMENT(Controller);
    FIND_CATEGORY_ELEMENT(Pedestrian);
    FIND_CATEGORY_ELEMENT(MiscObject);
    // FIND_CATEGORY_ELEMENT(Environment);
    FIND_CATEGORY_ELEMENT(Maneuver);
    // FIND_CATEGORY_ELEMENT(Trajectory);
    FIND_CATEGORY_ELEMENT(Route);
#undef FIND_CATEGORY_ELEMENT
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CATALOG_HPP_
