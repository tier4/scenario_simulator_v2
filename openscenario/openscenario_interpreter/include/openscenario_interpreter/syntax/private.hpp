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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PRIVATE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PRIVATE_HPP_

#include <nlohmann/json.hpp>
#include <openscenario_interpreter/syntax/private_action.hpp>
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Private ----------------------------------------------------------------
 *
 * <xsd:complexType name="Private">
 *   <xsd:sequence>
 *     <xsd:element name="PrivateAction" type="PrivateAction" maxOccurs="unbounded"/>
 *   </xsd:sequence>
 *   <xsd:attribute name="entityRef" type="String" use="required"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Private : public Scope
{
  const String entity_ref;

  std::list<PrivateAction> private_actions;

  template <typename Node>
  explicit Private(const Node & node, Scope & outer_scope)
  : Scope(outer_scope), entity_ref(readAttribute<String>("entityRef", node, localScope()))
  {
    actors.emplace_back(entity_ref);

    callWithElements(node, "PrivateAction", 1, unbounded, [&](auto && node) {
      return private_actions.emplace_back(node, localScope());
    });
  }

  auto endsImmediately() const -> bool;

  auto evaluate() -> Element;
};

auto operator<<(nlohmann::json &, const Private &) -> nlohmann::json &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PRIVATE_HPP_
