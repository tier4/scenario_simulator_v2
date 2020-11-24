// Copyright 2015-2020 TierIV.inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__CENTER_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__CENTER_HPP_

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ==== Center ===============================================================
 *
 * <xsd:complexType name="Center">
 *   <xsd:attribute name="x" type="Double" use="required"/>
 *   <xsd:attribute name="y" type="Double" use="required"/>
 *   <xsd:attribute name="z" type="Double" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Center
{
  const Double x, y, z;

  Center() = default;

  template<typename Node, typename Scope>
  explicit Center(const Node & node, Scope & scope)
  : x{readAttribute<Double>("x", node, scope)},
    y{readAttribute<Double>("y", node, scope)},
    z{readAttribute<Double>("z", node, scope)}
  {}
};

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const Center & rhs)
{
  return os << indent << blue << "<Center" << " " << highlight("x", rhs.x) <<
         " " << highlight("y", rhs.y) <<
         " " << highlight("z", rhs.z) << blue << "/>" << reset;
}
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CENTER_HPP_
