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

#ifndef SCENARIO_RUNNER__SYNTAX__ROUTE_HPP_
#define SCENARIO_RUNNER__SYNTAX__ROUTE_HPP_

#include <scenario_runner/syntax/parameter_declarations.hpp>
#include <scenario_runner/syntax/waypoint.hpp>

#include <string>
#include <utility>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== Route ================================================================
 *
 * <xsd:complexType name="Route">
 *   <xsd:sequence>
 *     <xsd:element name="ParameterDeclarations" type="ParameterDeclarations" minOccurs="0"/>
 *     <xsd:element name="Waypoint" minOccurs="2" maxOccurs="unbounded" type="Waypoint"/>
 *   </xsd:sequence>
 *   <xsd:attribute name="name" type="String" use="required"/>
 *   <xsd:attribute name="closed" type="Boolean" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Route
  : public Sequence
{
  const std::string name;

  const Boolean closed;

  template<typename Node, typename ... Ts>
  explicit Route(const Node & node, Ts && ... xs)
  : name{readRequiredAttribute<std::decay<decltype(name)>::type>(node, "name")},
    closed{readUnsupportedAttribute<std::decay<decltype(closed)>::type>(node, "closed", Boolean())}
  {
    defineElement<ParameterDeclarations>("ParameterDeclarations", 0, 1);
    defineElement<Waypoint>("Waypoint", 2, unbounded);

    validate(node, std::forward<decltype(xs)>(xs)...);
  }

  auto evaluate() const noexcept
  {
    return unspecified;
  }
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__ROUTE_HPP_
