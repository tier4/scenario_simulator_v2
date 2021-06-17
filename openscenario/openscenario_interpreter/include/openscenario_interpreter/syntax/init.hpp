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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__INIT_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__INIT_HPP_

#include <nlohmann/json.hpp>
#include <openscenario_interpreter/syntax/init_actions.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Init -------------------------------------------------------------------
 *
 *  <xsd:complexType name="Init">
 *    <xsd:sequence>
 *      <xsd:element name="Actions" type="InitActions"/>
 *    </xsd:sequence>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Init
{
  InitActions actions;

  template <typename Node, typename Scope>
  explicit Init(const Node & node, Scope & scope)
  : actions(readElement<InitActions>("Actions", node, scope))
  {
  }

  template <typename... Ts>
  decltype(auto) evaluate(Ts &&... xs)
  {
    return actions.evaluate(std::forward<decltype(xs)>(xs)...);
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__INIT_HPP_
