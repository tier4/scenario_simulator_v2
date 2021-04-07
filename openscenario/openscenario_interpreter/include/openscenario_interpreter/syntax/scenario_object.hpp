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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__SCENARIO_OBJECT_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__SCENARIO_OBJECT_HPP_

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/entity_object.hpp>
#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <openscenario_interpreter/syntax/object_controller.hpp>
#include <openscenario_interpreter/syntax/string.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ScenarioObject ---------------------------------------------------------
 *
 *  <xsd:complexType name="ScenarioObject">
 *    <xsd:sequence>
 *      <xsd:group ref="EntityObject"/>
 *      <xsd:element name="ObjectController" minOccurs="0" type="ObjectController"/>
 *    </xsd:sequence>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ScenarioObject
/* -----------------------------------------------------------------------------
 *
 *  The EntityObject (either instance of type Vehicle, Pedestrian or
 *  MiscObject).
 *
 *  NOTE: This framework expresses xsd:group as mixin.
 *
 * ------------------------------------------------------------------------- */
: public EntityObject
{
  /* ---- name -----------------------------------------------------------------
   *
   *  Identifier of the scenario object.
   *
   * ------------------------------------------------------------------------ */
  using Name = String;

  const Name name;

  /* ---- ObjectController -----------------------------------------------------
   *
   *  Controller of the EntityObject instance.
   *
   * ------------------------------------------------------------------------ */
  ObjectController object_controller;

  static_assert(IsOptionalElement<ObjectController>::value, "minOccurs=\"0\"");

  template <typename Node, typename Scope>
  explicit ScenarioObject(const Node & node, Scope & outer_scope)
  : EntityObject(node, outer_scope),
    name(readAttribute<String>("name", node, outer_scope)),
    object_controller(readElement<ObjectController>("ObjectController", node, outer_scope))
  {
  }

  auto evaluate()
  {
    if (spawn(
          is<Vehicle>() && object_controller.isEgo(), name,
          boost::lexical_cast<String>(
            static_cast<const EntityObject &>(*this))))  // XXX UGLY CODE!!!
    {
      if (is<Vehicle>()) {
        setController(name, object_controller);

        if (object_controller.isEgo()) {
          attachLidarSensor(traffic_simulator::helper::constructLidarConfiguration(
            traffic_simulator::helper::LidarType::VLP32, name,
            "/sensing/lidar/no_ground/pointcloud"));
          attachDetectionSensor(traffic_simulator::helper::constructDetectionSensorConfiguration(
            name, "/perception/object_recognition/objects", 0.1));
        }
      }
      return unspecified;
    } else {
      throw SemanticError("Failed to spawn entity '", name, "'.");
    }
  }
};

std::ostream & operator<<(std::ostream & os, const ScenarioObject & datum)
{
  return os << (indent++) << blue << "<ScenarioObject"
            << " " << highlight("name", datum.name) << blue << ">\n"
            << reset << static_cast<const EntityObject &>(datum) << "\n"
            << (--indent) << blue << "</ScenarioObject>" << reset;
}
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__SCENARIO_OBJECT_HPP_
