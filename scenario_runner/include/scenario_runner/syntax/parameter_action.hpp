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

#ifndef SCENARIO_RUNNER__SYNTAX__PARAMETER_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__PARAMETER_ACTION_HPP_

#include <scenario_runner/reader/attribute.hpp>
#include <scenario_runner/reader/element.hpp>
#include <scenario_runner/syntax/modify_action.hpp>
#include <scenario_runner/syntax/parameter_set_action.hpp>

#include <utility>

namespace scenario_runner
{
inline namespace syntax
{
/* ---- ParameterAction --------------------------------------------------------
 *
 * <xsd:complexType name="ParameterAction">
 *   <xsd:choice>
 *     <xsd:element name="SetAction" type="ParameterSetAction"/>
 *     <xsd:element name="ModifyAction" type="ParameterModifyAction"/>
 *   </xsd:choice>
 *   <xsd:attribute name="parameterRef" type="String" use="required"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ParameterAction
  : public Element
{
  template<typename Node, typename Scope>
  explicit ParameterAction(const Node & parent, Scope & scope)
  : Element(
      choice(
        parent,

        std::make_pair("SetAction", [&](auto && child)
        {
          return make<ParameterSetAction>(
            child, scope, readAttribute<String>("parameterRef", parent, scope));
        }),

        std::make_pair("ModifyAction", [&](auto && child)
        {
          return make<ModifyAction>(
            child, scope, readAttribute<String>("parameterRef", parent, scope));
        })))
  {}
};
}  // inline namespace syntax
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__PARAMETER_ACTION_HPP_
