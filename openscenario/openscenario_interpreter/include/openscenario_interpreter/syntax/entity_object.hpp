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

#include <openscenario_interpreter/syntax/pedestrian.hpp>
#include <openscenario_interpreter/syntax/vehicle.hpp>

#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
#define ELEMENT(TYPE) \
  std::make_pair( \
    #TYPE, [&](auto && node) \
    { \
      return make<TYPE>(node, std::forward<decltype(xs)>(xs)...); \
    })

/* ---- EntityObject -----------------------------------------------------------
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
 * -------------------------------------------------------------------------- */
struct EntityObject
  : public Group
{
  template
  <
    typename Node, typename ... Ts
  >
  explicit EntityObject(const Node & node, Ts && ... xs)
  : Group(
      choice(
        node,
        std::make_pair("CatalogReference", UNSUPPORTED()),
        ELEMENT(Vehicle),
        ELEMENT(Pedestrian),
        std::make_pair("MiscObject", UNSUPPORTED())))
  {}
};

#undef ELEMENT
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_OBJECT_HPP_
