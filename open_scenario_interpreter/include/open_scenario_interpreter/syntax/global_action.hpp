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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__GLOBAL_ACTION_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__GLOBAL_ACTION_HPP_

#include <open_scenario_interpreter/syntax/infrastructure_action.hpp>
#include <open_scenario_interpreter/syntax/parameter_action.hpp>

#include <utility>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ---- GlobalAction -----------------------------------------------------------
 *
 * <xsd:complexType name="GlobalAction">
 *   <xsd:choice>
 *     <xsd:element name="EnvironmentAction" type="EnvironmentAction"/>
 *     <xsd:element name="EntityAction" type="EntityAction"/>
 *     <xsd:element name="ParameterAction" type="ParameterAction"/>
 *     <xsd:element name="InfrastructureAction" type="InfrastructureAction"/>
 *     <xsd:element name="TrafficAction" type="TrafficAction"/>
 *   </xsd:choice>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct GlobalAction
  : public Element
{
  template<typename Node, typename ... Ts>
  explicit GlobalAction(const Node & node, Ts && ... xs)
  : Element(
      choice(
        node,
        std::make_pair("EnvironmentAction", UNSUPPORTED()),
        std::make_pair("EntityAction", UNSUPPORTED()),

        std::make_pair("ParameterAction", [&](auto && node)
        {
          return make<ParameterAction>(node, std::forward<decltype(xs)>(xs)...);
        }),

        std::make_pair("InfrastructureAction", UNSUPPORTED()),
        std::make_pair("TrafficAction", UNSUPPORTED())))
  {}
};
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__GLOBAL_ACTION_HPP_
