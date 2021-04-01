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

#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- AssignRouteAction ------------------------------------------------------
 *
 *  <xsd:complexType name="AssignRouteAction">
 *    <xsd:choice>
 *      <xsd:element name="Route" type="Route"/>
 *      <xsd:element name="CatalogReference" type="CatalogReference"/>
 *    </xsd:choice>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct AssignRouteAction
{
  Scope inner_scope;

  Element route_or_catalog_reference;

  template<typename Node>
  explicit AssignRouteAction(const Node & node, Scope & outer_scope)
  : inner_scope(outer_scope),
    route_or_catalog_reference(
      choice(
        node,
        std::make_pair("Route", [&](auto && node) {
          return make<Route>(node, inner_scope);
        }),
        std::make_pair("CatalogReference", UNSUPPORTED())))
  {}

  const std::true_type accomplished {};

  decltype(auto) operator()(const Scope::Actor & actor)
  {
    return requestAssignRoute(
      actor,
      static_cast<
        std::vector<openscenario_msgs::msg::LaneletPose>
      >(route_or_catalog_reference.as<const Route>()));
  }

  // auto start()
  // {
  //   for (const auto & actor : inner_scope.actors) {
  //   }
  // }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ASSIGN_ROUTE_ACTION_HPP_
