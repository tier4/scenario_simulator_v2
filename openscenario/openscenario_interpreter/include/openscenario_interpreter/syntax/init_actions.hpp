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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__INIT_ACTIONS_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__INIT_ACTIONS_HPP_

#include <openscenario_interpreter/syntax/action.hpp>
#include <openscenario_interpreter/syntax/private.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include "openscenario_interpreter/syntax/global_action.hpp"
#include "openscenario_interpreter/syntax/user_defined_action.hpp"

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- InitActions ------------------------------------------------------------
 *
 * <xsd:complexType name="InitActions">
 *   <xsd:sequence>
 *     <xsd:element name="GlobalAction" type="GlobalAction" minOccurs="0" maxOccurs="unbounded"/>
 *     <xsd:element name="UserDefinedAction" type="UserDefinedAction" minOccurs="0" maxOccurs="unbounded"/>
 *     <xsd:element name="Private" minOccurs="0" maxOccurs="unbounded" type="Private"/>
 *   </xsd:sequence>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
#define ELEMENT(TYPE) \
  std::make_pair(#TYPE, [&](auto && node) { push_back(make<TYPE>(node, scope)); })

struct InitActions : public Elements
{
  template <typename Node, typename Scope>
  explicit InitActions(const Node & node, Scope & scope)
  {
    std::unordered_map<std::string, std::function<void(const Node & node)>> dispatcher{
      ELEMENT(GlobalAction),
      ELEMENT(UserDefinedAction),
      ELEMENT(Private),
    };

    for (const auto & each : node.children()) {
      const auto iter{dispatcher.find(each.name())};
      if (iter != std::end(dispatcher)) {
        std::get<1> (*iter)(each);
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

  bool is_complete_immediately() const
  {
    return std::all_of(begin(), end(), [](const Element & elm) {
      if (elm.is<GlobalAction>()) {
        return elm.as<GlobalAction>().is_complete_immediately();
      } else if (elm.is<UserDefinedAction>()) {
        return elm.as<UserDefinedAction>().is_complete_immediately();
      } else if (elm.is<Private>()) {
        return elm.as<Private>().is_complete_immediately();
      }
    });
  }
};

#undef ELEMENT
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__INIT_ACTIONS_HPP_
