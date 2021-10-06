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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_TARGET_SPEED_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_TARGET_SPEED_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/boolean.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <openscenario_interpreter/syntax/speed_target_value_type.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- RelativeTargetSpeed ----------------------------------------------------
 *
 *  <xsd:complexType name="RelativeTargetSpeed">
 *    <xsd:attribute name="entityRef" type="String" use="required"/>
 *    <xsd:attribute name="value" type="Double" use="required"/>
 *    <xsd:attribute name="speedTargetValueType" type="SpeedTargetValueType" use="required"/>
 *    <xsd:attribute name="continuous" type="Boolean" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct RelativeTargetSpeed
{
  const String entity_ref;

  const Double value;

  const SpeedTargetValueType speed_target_value_type;

  const Boolean continuous;

  explicit RelativeTargetSpeed(const pugi::xml_node &, Scope &);

  auto getCalculateAbsoluteTargetSpeed() const -> std::function<double()>;

  auto getIsEnd() const -> std::function<bool(const EntityRef &)>;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_TARGET_SPEED_HPP_
