// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ENTITIES_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ENTITIES_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/entity.hpp>
#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   Entities (OpenSCENARIO XML 1.3.1)

   Definition of entities (scenario objects or entity selections) in
   a scenario.

   <xsd:complexType name="Entities">
     <xsd:sequence>
       <xsd:element name="ScenarioObject" type="ScenarioObject" minOccurs="0" maxOccurs="unbounded"/>
       <xsd:element name="EntitySelection" type="EntitySelection" minOccurs="0" maxOccurs="unbounded"/>
     </xsd:sequence>
   </xsd:complexType>
*/
struct Entities : private std::unordered_map<std::string, Object>
{
  using std::unordered_map<std::string, Object>::at;
  using std::unordered_map<std::string, Object>::begin;
  using std::unordered_map<std::string, Object>::end;

  explicit Entities(const pugi::xml_node &, Scope &);

  auto isAdded(const Entity &) const -> bool;

  auto ref(const EntityRef &) const -> Object;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ENTITIES_HPP_
