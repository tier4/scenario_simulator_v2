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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__BOUNDING_BOX_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__BOUNDING_BOX_HPP_

#include <openscenario_interpreter/syntax/center.hpp>
#include <openscenario_interpreter/syntax/dimensions.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ==== BoundingBox ==========================================================
 *
 * <xsd:complexType name="BoundingBox">
 *   <xsd:all>
 *     <xsd:element name="Center" type="Center"/>
 *     <xsd:element name="Dimensions" type="Dimensions"/>
 *   </xsd:all>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct BoundingBox
{
  const Center center;

  const Dimensions dimensions;

  BoundingBox() = default;

  template<typename Node, typename Scope>
  explicit BoundingBox(const Node & node, Scope & scope)
  : center{readElement<Center>("Center", node, scope)},
    dimensions{readElement<Dimensions>("Dimensions", node, scope)}
  {}
};

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const BoundingBox & rhs)
{
  return os << (indent++) << blue << "<BoundingBox>\n" << reset <<
         rhs.center << "\n" <<
         rhs.dimensions << "\n" <<
         (--indent) << blue << "</BoundingBox>" << reset;
}
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__BOUNDING_BOX_HPP_
