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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PROPERTIES_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PROPERTIES_HPP_

#include <openscenario_interpreter/syntax/file.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Properties -------------------------------------------------------------
 *
 *  <xsd:complexType name="Properties">
 *    <xsd:sequence>
 *      <xsd:element name="Property" minOccurs="0" maxOccurs="unbounded" type="Property"/>
 *      <xsd:element name="File" type="File" minOccurs="0" maxOccurs="unbounded"/>
 *    </xsd:sequence>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Properties
{
  Properties() = default;

  // std::unordered_map<String, Property> properties;

  template
  <
    typename Node, typename Scope
  >
  explicit Properties(const Node & node, Scope & outer_scope)
  {
    // callWithElements(
    //   node, "Property", 0, unbounded, [this](auto && each)
    //   {
    //     return properties.emplace(
    //       readAttribute<String>("name", each, outer_scope), each, outer_scope);
    //   });
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PROPERTIES_HPP_
