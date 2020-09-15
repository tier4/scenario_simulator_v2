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

#ifndef SCENARIO_RUNNER__SYNTAX__INIT_ACTIONS_HPP_
#define SCENARIO_RUNNER__SYNTAX__INIT_ACTIONS_HPP_

#include <scenario_runner/syntax/action.hpp>
#include <scenario_runner/syntax/private.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== InitActions ==========================================================
 *
 * <xsd:complexType name="InitActions">
 *   <xsd:sequence>
 *     <xsd:element name="GlobalAction" type="GlobalAction" minOccurs="0" maxOccurs="unbounded"/>
 *     <xsd:element name="UserDefinedAction" type="UserDefinedAction" minOccurs="0" maxOccurs="unbounded"/>
 *     <xsd:element name="Private" minOccurs="0" maxOccurs="unbounded" type="Private"/>
 *   </xsd:sequence>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct InitActions
  : public std::vector<Object>
{
  template<typename Node, typename Scope>
  explicit InitActions(const Node & node, Scope & scope)
  {
    std::unordered_map<std::string, std::function<void(const Node & node)>> dispatcher
    {
      std::make_pair("GlobalAction",
        [&](auto && node) {push_back(make<GlobalAction>(node, scope));}),
      std::make_pair("UserDefinedAction", [&](auto && node) {
          push_back(make<UserDefinedAction>(node, scope));
        }),
      std::make_pair("Private", [&](auto && node) {push_back(make<Private>(node, scope));}),
    };

    for (const auto & each : node.children()) {
      const auto iter {dispatcher.find(each.name())};

      if (iter != std::end(dispatcher)) {
        std::get<1>(* iter)(each);
      }
    }
  }

  auto evaluate() const
  {
    for (auto && each : *this) {
      each.evaluate();
    }

    return unspecified;
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__INIT_ACTIONS_HPP_
