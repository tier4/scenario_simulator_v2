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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ASSIGN_ROUTE_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ASSIGN_ROUTE_ACTION_HPP_

#include <openscenario_interpreter/syntax/route.hpp>

#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ==== AssignRouteAction ====================================================
 *
 * <xsd:complexType name="AssignRouteAction">
 *   <xsd:choice>
 *     <xsd:element name="Route" type="Route"/>
 *     <xsd:element name="CatalogReference" type="CatalogReference"/>
 *   </xsd:choice>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct AssignRouteAction
  : public Element
{
  template<typename Node, typename ... Ts>
  explicit AssignRouteAction(const Node & node, Ts && ... xs)
  : Element(
      choice(
        node,
        std::make_pair("Route", [&](auto && node) {
          return make<Route>(node, std::forward<decltype(xs)>(xs)...);
        }),
        std::make_pair("CatalogReference", UNSUPPORTED())))
  {}
};
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ASSIGN_ROUTE_ACTION_HPP_
