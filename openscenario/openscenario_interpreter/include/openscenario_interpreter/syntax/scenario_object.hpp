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

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ScenarioObject ---------------------------------------------------------
 *
 * <xsd:complexType name="ScenarioObject">
 *   <xsd:sequence>
 *     <xsd:group ref="EntityObject"/>
 *     <xsd:element name="ObjectController" minOccurs="0" type="ObjectController"/>
 *   </xsd:sequence>
 *   <xsd:attribute name="name" type="String" use="required"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ScenarioObject
{
  const String name;

  Element entity_object, object_controller;

  template<typename Node>
  explicit ScenarioObject(const Node & node, Scope & outer_scope)
  : name(readAttribute<String>("name", node, outer_scope)),
    entity_object(make<EntityObject>(node, outer_scope))
  {
    callWithElements(
      node, "ObjectController", 0, 1, [&](auto && node)
      {
        return object_controller.rebind<ObjectController>(node, outer_scope);
      });
  }

  decltype(auto) getEntityObject()
  {
    return entity_object.as<EntityObject>(__FILE__, __LINE__);
  }

  auto evaluate()
  {
    return asBoolean(spawn(false, name, boost::lexical_cast<String>(entity_object)));
  }
};

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const ScenarioObject & rhs)
{
  return os << (indent++) << blue << "<ScenarioObject" << " " <<
         highlight("name", rhs.name) << blue << ">\n" << reset <<
         rhs.entity_object << "\n" <<
         (--indent) << blue << "</ScenarioObject>" << reset;
}
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__SCENARIO_OBJECT_HPP_
