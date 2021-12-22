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

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/file.hpp>
#include <openscenario_interpreter/syntax/property.hpp>
#include <pugixml.hpp>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Properties -------------------------------------------------------------
 *
 *  Container for one or more properties. Properties encloses multiple property
 *  instances and/or multiple file references.
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
  /*
     A name-value pair. The semantic of the name/values are subject of a
     contract between the provider of a simulation environment and the author
     of a scenario.
  */
  std::unordered_map<String, Property> properties;

  /*
     A list of arbitrary files attached to an object that owns the properties.
     The semantic and the file formats are subject of a contract between the
     provider of a simulation environment and the author of a scenario.

     NOTE: currently ignored.
  */
  std::list<File> files;

  Properties() = default;

  explicit Properties(const pugi::xml_node &, Scope &);

#define BOILERPLATE(NAME)                                      \
  template <typename... Ts>                                    \
  auto NAME(Ts &&... xs)->decltype(auto)                       \
  {                                                            \
    return properties.NAME(std::forward<decltype(xs)>(xs)...); \
  }                                                            \
  static_assert(true)

  BOILERPLATE(end);
  BOILERPLATE(find);
  BOILERPLATE(operator[]);

#undef BOILERPLATE
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PROPERTIES_HPP_
