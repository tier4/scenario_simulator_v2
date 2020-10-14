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

#ifndef SCENARIO_RUNNER__SYNTAX__SCENARIO_OBJECT_HPP_
#define SCENARIO_RUNNER__SYNTAX__SCENARIO_OBJECT_HPP_

#include <scenario_runner/syntax/entity_object.hpp>
#include <scenario_runner/syntax/entity_ref.hpp>
#include <scenario_runner/syntax/object_controller.hpp>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== ScenarioObject =======================================================
 *
 * <xsd:complexType name="ScenarioObject">
 *   <xsd:sequence>
 *     <xsd:group ref="EntityObject"/>
 *     <xsd:element name="ObjectController" minOccurs="0" type="ObjectController"/>
 *   </xsd:sequence>
 *   <xsd:attribute name="name" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct ScenarioObject
{
  const String name;

  Scope inner_scope;

  Element entity_object, object_controller;

  template<typename Node>
  explicit ScenarioObject(const Node & node, Scope & outer_scope)
  : name{readAttribute<String>("name", node, outer_scope)},
    inner_scope{outer_scope},
    entity_object{make<EntityObject>(node, inner_scope)}
  {
    callWithElements(
      node, "ObjectController", 0, 1, [&](auto && node)
      {
        return object_controller.rebind<ObjectController>(node, inner_scope);
      });
  }

  decltype(auto) getEntityObject()
  {
    return entity_object.as<EntityObject>(__FILE__, __LINE__);
  }

  auto evaluate()
  {
    std::cout << indent << "spawn(false, " << name << ", ...)\n" << entity_object << std::endl;
    // return asBoolean(inner_scope.connection->entity->spawn(
    // false, name, boost::lexical_cast<String>(entity_object)));
    return unspecified;
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
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__SCENARIO_OBJECT_HPP_
