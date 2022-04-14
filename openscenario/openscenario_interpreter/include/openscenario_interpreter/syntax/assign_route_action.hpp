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

#include <openscenario_interpreter/scope.hpp>
#include <pugixml.hpp>

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
struct AssignRouteAction : private Scope
{
  Object route;

  explicit AssignRouteAction(const pugi::xml_node &, Scope &);

  static auto accomplished() noexcept -> bool;

  static auto endsImmediately() noexcept -> bool;

  static auto run() -> void;

  /*  */ auto start() -> void;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ASSIGN_ROUTE_ACTION_HPP_
