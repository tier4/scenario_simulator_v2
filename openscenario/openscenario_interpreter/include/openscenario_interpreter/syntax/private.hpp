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

  auto evaluate()
  {
    for (auto && private_action : private_actions) {
      // NOTE: standbyState -> startTransition (if ready)
      // private_action.ready();

      // NOTE: startTransition -> runningState (unconditionally)
      private_action.start();

      // NOTE: runningState -> endTransition (if accomplished)
      do {
        private_action.run();
      } while (not private_action.accomplished());

      // NOTE: endTransition -> completeState (Init.Actions only once executed)
    }

    return unspecified;
  }

  bool endsImmediately() const
  {
    return std::all_of(
      private_actions.begin(), private_actions.end(),
      [](const PrivateAction & private_action) { return private_action.endsImmediately(); });
  }
};

nlohmann::json & operator<<(nlohmann::json &, const Private &);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PRIVATE_HPP_
