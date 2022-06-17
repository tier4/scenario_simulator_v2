// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__USER_DEFINED_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__USER_DEFINED_ACTION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/custom_command_action.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- UserDefinedAction ------------------------------------------------------
 *
 *  <xsd:complexType name="UserDefinedAction">
 *    <xsd:sequence>
 *      <xsd:element name="CustomCommandAction" type="CustomCommandAction"/>
 *    </xsd:sequence>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct UserDefinedAction : public ComplexType
{
  explicit UserDefinedAction(const pugi::xml_node &, Scope &);

  static auto endsImmediately() -> bool;

  /*  */ auto evaluate() -> Object;

  /*  */ auto run() -> void;

  /*  */ auto start() -> void;
};

DEFINE_LAZY_VISITOR(
  UserDefinedAction,
  CASE(CustomCommandAction),  //
);

DEFINE_LAZY_VISITOR(
  const UserDefinedAction,
  CASE(CustomCommandAction),  //
);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__USER_DEFINED_ACTION_HPP_
