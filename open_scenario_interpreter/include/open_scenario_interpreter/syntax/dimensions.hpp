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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__DIMENSIONS_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__DIMENSIONS_HPP_

#include <open_scenario_interpreter/reader/attribute.hpp>
#include <open_scenario_interpreter/reader/element.hpp>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ==== Dimensions ===========================================================
 *
 * <xsd:complexType name="Dimensionss">
 *   <xsd:attribute name="width" type="Double" use="required"/>
 *   <xsd:attribute name="length" type="Double" use="required"/>
 *   <xsd:attribute name="height" type="Double" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Dimensions
{
  const Double width, length, height;

  Dimensions() = default;

  template<typename Node, typename Scope>
  explicit Dimensions(const Node & node, Scope & scope)
  : width{readAttribute<Double>("width", node, scope)},
    length{readAttribute<Double>("length", node, scope)},
    height{readAttribute<Double>("height", node, scope)}
  {}
};

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const Dimensions & rhs)
{
  return os << indent << blue << "<Dimensions" << " " << highlight("width", rhs.width) <<
         " " << highlight("length", rhs.length) <<
         " " << highlight("height", rhs.height) << blue << "/>" << reset;
}
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__DIMENSIONS_HPP_
