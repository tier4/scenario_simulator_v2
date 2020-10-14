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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__ENTITY_OBJECT_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__ENTITY_OBJECT_HPP_

#include <open_scenario_interpreter/syntax/pedestrian.hpp>
#include <open_scenario_interpreter/syntax/vehicle.hpp>

#include <utility>

namespace open_scenario_interpreter
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
  template<typename Node, typename ... Ts>
  explicit EntityObject(const Node & node, Ts && ... xs)
  : Element(
      choice(
        node,
        std::make_pair("CatalogReference", UNSUPPORTED()),
        std::make_pair("Vehicle", [&](auto && node) {
          return make<Vehicle>(node, std::forward<decltype(xs)>(xs)...);
        }),
        std::make_pair("Pedestrian", [&](auto && node) {
          return make<Pedestrian>(node, std::forward<decltype(xs)>(xs)...);
        }),
        std::make_pair("MiscObject", UNSUPPORTED())))
  {}
};
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__ENTITY_OBJECT_HPP_
