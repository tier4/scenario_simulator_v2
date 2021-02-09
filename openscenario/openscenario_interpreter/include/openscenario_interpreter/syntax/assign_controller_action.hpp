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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ASSIGN_CONTROLLER_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ASSIGN_CONTROLLER_ACTION_HPP_

#include <openscenario_interpreter/syntax/controller.hpp>

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

/* ---- AssignControllerAction -------------------------------------------------
 *
 *  This action assigns a controller to the given entity defined in the
 *  enclosing PrivateAction. Controllers could be defined inline or by using a
 *  catalog reference.
 *
 *  <xsd:complexType name="AssignControllerAction">
 *    <xsd:choice>
 *      <xsd:element name="Controller" type="Controller"/>
 *      <xsd:element name="CatalogReference" type="CatalogReference"/>
 *    </xsd:choice>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct AssignControllerAction : public ComplexType
{
  template
  <
    typename Node,
    typename ... Ts
  >
  explicit AssignControllerAction(const Node & node, Ts && ... xs)
  : ComplexType(
      choice(
        node,
        ELEMENT(Controller),
        std::make_pair("CatalogReference", UNSUPPORTED())))
};

#undef ELEMENT
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ASSIGN_CONTROLLER_ACTION_HPP_
