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

#ifndef SCENARIO_RUNNER__SYNTAX__ASSIGN_ROUTE_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__ASSIGN_ROUTE_ACTION_HPP_

#include <scenario_runner/syntax/route.hpp>

namespace scenario_runner
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
  template<typename Node, typename Scope>
  explicit AssignRouteAction(const Node & node, Scope & scope)
  {
    callWithElements(
      node, "Route", 0, 1, [&](auto && node)
      {
        return rebind<Route>(node, scope);
      });

    callWithElements(
      node, "CatalogReference", 0, 1, THROW_UNSUPPORTED_ERROR(node));
  }
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__ASSIGN_ROUTE_ACTION_HPP_
