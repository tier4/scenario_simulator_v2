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
#include <openscenario_interpreter/syntax/property.hpp>

#include <unordered_map>
#include <utility>
#include <vector>

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

  std::unordered_map<Property::Name, Property> properties;

  std::vector<File> files;

  template
  <
    typename Node, typename Scope
  >
  explicit Properties(const Node & node, Scope & outer_scope)
  {
    callWithElements(
      node, "Property", 0, unbounded, [&](auto && node)
      {
        return properties.emplace(
          std::piecewise_construct,
          std::forward_as_tuple(readAttribute<Property::Name>("name", node, outer_scope)),
          std::forward_as_tuple(node, outer_scope));
      });

    callWithElements(
      node, "File", 0, unbounded, [&](auto && node)
      {
        return files.emplace_back(std::forward<decltype(node)>(node), outer_scope);
      });
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PROPERTIES_HPP_
