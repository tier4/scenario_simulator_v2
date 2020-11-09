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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PRIVATE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PRIVATE_HPP_

#include <openscenario_interpreter/syntax/private_action.hpp>

#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ==== Private ==============================================================
 *
 * <xsd:complexType name="Private">
 *   <xsd:sequence>
 *     <xsd:element name="PrivateAction" type="PrivateAction" maxOccurs="unbounded"/>
 *   </xsd:sequence>
 *   <xsd:attribute name="entityRef" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Private
  : public std::vector<PrivateAction>
{
  Scope inner_scope;

  template<typename Node>
  explicit Private(const Node & node, Scope & outer_scope)
  : inner_scope{outer_scope}
  {
    inner_scope.actors.emplace_back(
      readAttribute<String>("entityRef", node, inner_scope));

    callWithElements(
      node, "PrivateAction", 1, unbounded, [&](auto && node)
      {
        emplace_back(node, inner_scope);
      });
  }

  auto evaluate()
  {
    for (auto && each : *this) {
      each.start();
    }

    return unspecified;
  }
};
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PRIVATE_HPP_
