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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_ACTION_HPP_

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/parameter_modify_action.hpp>
#include <openscenario_interpreter/syntax/parameter_set_action.hpp>
#include <utility>

namespace openscenario_interpreter
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
#define ELEMENT(NAME)                                                                  \
  std::make_pair(#NAME, [&](auto && child) {                                           \
    return make<Parameter##NAME>(                                                      \
      child, outer_scope, readAttribute<String>("parameterRef", parent, outer_scope)); \
  })

struct ParameterAction : public Element
{
  template <typename Node, typename Scope>
  explicit ParameterAction(const Node & parent, Scope & outer_scope)
  : Element(choice(parent, ELEMENT(SetAction), ELEMENT(ModifyAction)))
  {
  }

  static constexpr std::true_type is_complete_immediately{};
};

#undef ELEMENT
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_ACTION_HPP_
