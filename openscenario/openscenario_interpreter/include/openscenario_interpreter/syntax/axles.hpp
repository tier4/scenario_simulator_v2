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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__AXLES_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__AXLES_HPP_

#include <openscenario_interpreter/syntax/axle.hpp>

#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ==== Axles ================================================================
 *
 * <xsd:complexType name="Axles">
 *   <xsd:sequence>
 *     <xsd:element name="FrontAxle" type="Axle"/>
 *     <xsd:element name="RearAxle" type="Axle"/>
 *     <xsd:element name="AdditionalAxle" type="Axle" minOccurs="0" maxOccurs="unbounded"/>
 *   </xsd:sequence>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Axles
{
  const FrontAxle front_axle;

  const RearAxle rear_axle;

  std::vector<AdditionalAxle> additional_axles;

  Axles() = default;

  template<typename Node, typename Scope>
  explicit Axles(const Node & node, Scope & scope)
  : front_axle{readElement<FrontAxle>("FrontAxle", node, scope)},
    rear_axle{readElement<RearAxle>("RearAxle", node, scope)}
  {
    callWithElements(node, "AdditionalAxle", 0, unbounded, [&](auto && node)
      {
        additional_axles.emplace_back(node, scope);
      });
  }
};

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const Axles & rhs)
{
  os << (indent++) << blue << "<Axles>\n" << reset <<
    rhs.front_axle << "\n" <<
    rhs.rear_axle << "\n";

  for (const auto & each : rhs.additional_axles) {
    os << each << "\n";
  }

  return os << (--indent) << blue << "</Axles>" << reset;
}
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__AXLES_HPP_
