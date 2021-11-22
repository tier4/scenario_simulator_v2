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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_OBJECT_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_OBJECT_HPP_

#include <openscenario_interpreter/syntax/catalog_reference.hpp>
#include <openscenario_interpreter/syntax/misc_object.hpp>
#include <openscenario_interpreter/syntax/pedestrian.hpp>
#include <openscenario_interpreter/syntax/vehicle.hpp>
#include <unordered_map>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- EntityObject -----------------------------------------------------------
 *
 *  <xsd:group name="EntityObject">
 *    <xsd:choice>
 *      <xsd:element name="CatalogReference" type="CatalogReference"/>
 *      <xsd:element name="Vehicle" type="Vehicle"/>
 *      <xsd:element name="Pedestrian" type="Pedestrian"/>
 *      <xsd:element name="MiscObject" type="MiscObject"/>
 *    </xsd:choice>
 *  </xsd:group>
 *
 * -------------------------------------------------------------------------- */
struct EntityObject : public Group
{
  template <typename XML, typename... Ts>
  explicit EntityObject(const XML & node, Ts &&... xs)
  // clang-format off
  : Group(
      choice(node,
        std::make_pair("CatalogReference", [&](auto && node) { return CatalogReference::make<Vehicle, Pedestrian, MiscObject>(node, std::forward<decltype(xs)>(xs)...); }),
        std::make_pair("Vehicle",          [&](auto && node) { return make<Vehicle>         (node, std::forward<decltype(xs)>(xs)...); }),
        std::make_pair("Pedestrian",       [&](auto && node) { return make<Pedestrian>      (node, std::forward<decltype(xs)>(xs)...); }),
        std::make_pair("MiscObject",       [&](auto && node) { return make<MiscObject>      (node, std::forward<decltype(xs)>(xs)...); })))
  // clang-format on
  {
  }
};

DEFINE_LAZY_VISITOR(
  EntityObject,
  // CASE(CatalogReference),  //
  CASE(Vehicle),     //
  CASE(Pedestrian),  //
  CASE(MiscObject),  //
);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_OBJECT_HPP_
