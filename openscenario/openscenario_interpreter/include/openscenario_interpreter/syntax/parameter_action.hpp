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

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/parameter_modify_action.hpp>
#include <openscenario_interpreter/syntax/parameter_set_action.hpp>
#include <pugixml.hpp>

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
struct ParameterAction : public Element
{
  explicit ParameterAction(const pugi::xml_node &, Scope &);

  static auto endsImmediately() -> bool;

  /*  */ auto run() -> void;

  /*  */ auto start() -> void;
};

DEFINE_LAZY_VISITOR(
  ParameterAction,
  CASE(ParameterSetAction),     //
  CASE(ParameterModifyAction),  //
);

DEFINE_LAZY_VISITOR(
  const ParameterAction,
  CASE(ParameterSetAction),     //
  CASE(ParameterModifyAction),  //
);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_ACTION_HPP_
