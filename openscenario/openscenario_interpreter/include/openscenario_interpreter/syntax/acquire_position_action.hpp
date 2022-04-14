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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ACQUIRE_POSITION_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ACQUIRE_POSITION_ACTION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/position.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- AcquirePositionAction --------------------------------------------------
 *
 *  <xsd:complexType name="AcquirePositionAction">
 *    <xsd:all>
 *      <xsd:element name="Position" type="Position"/>
 *    </xsd:all>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct AcquirePositionAction : private Scope
{
  const Position position;

  explicit AcquirePositionAction(const pugi::xml_node &, Scope &);

  static constexpr auto accomplished() -> bool { return true; }

  static constexpr auto endsImmediately() -> bool { return true; }

  static constexpr auto run() noexcept -> void {}

  auto start() -> void;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ACQUIRE_POSITION_ACTION_HPP_
