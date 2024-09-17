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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_REF_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_REF_HPP_

#include <openscenario_interpreter/reader/attribute.hpp>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   EntityRef (OpenSCENARIO XML 1.3)

   Unambiguous reference to an entity represented as its name.

   <xsd:complexType name="EntityRef">
     <xsd:attribute name="entityRef" type="String" use="required"/>
   </xsd:complexType>
*/
struct EntityRef : public String
{
  EntityRef(const String & string) : String(string) {}

  template <typename Node, typename Scope>
  explicit EntityRef(const Node & node, Scope & scope)
  : String(readAttribute<String>("entityRef", node, scope))
  {
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_REF_HPP_
