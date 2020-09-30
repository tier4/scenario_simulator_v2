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

#ifndef SCENARIO_RUNNER__SYNTAX__ENTITY_OBJECT_HPP_
#define SCENARIO_RUNNER__SYNTAX__ENTITY_OBJECT_HPP_

#include <scenario_runner/syntax/pedestrian.hpp>
#include <scenario_runner/syntax/vehicle.hpp>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== EntityObject =========================================================
 *
 * <xsd:group name="EntityObject">
 *   <xsd:choice>
 *     <xsd:element name="CatalogReference" type="CatalogReference"/>
 *     <xsd:element name="Vehicle" type="Vehicle"/>
 *     <xsd:element name="Pedestrian" type="Pedestrian"/>
 *     <xsd:element name="MiscObject" type="MiscObject"/>
 *   </xsd:choice>
 * </xsd:group>
 *
 * ======================================================================== */
struct EntityObject
  : public Element
{
  template<typename Node>
  explicit EntityObject(const Node & node, Scope & scope)
  {
    callWithElements(node, "CatalogReference", 0, 1, THROW_UNSUPPORTED_ERROR(node));

    callWithElements(
      node, "Vehicle", 0, 1, [&](auto && element)
      {
        return rebind<Vehicle>(element, scope);
      });

    callWithElements(
      node, "Pedestrian", 0, 1, [&](auto && element) mutable
      {
        return rebind<Pedestrian>(element, scope);
      });

    callWithElements(node, "MiscObject", 0, 1, THROW_UNSUPPORTED_ERROR(node));
  }
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__ENTITY_OBJECT_HPP_
