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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ABSOLUTE_TARGET_SPEED_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ABSOLUTE_TARGET_SPEED_HPP_

#include <openscenario_interpreter/reader/attribute.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ==== AbsoluteTargetSpeed ====================================================
 *
 * <xsd:complexType name="AbsoluteTargetSpeed">
 *   <xsd:attribute name="value" type="Double" use="required"/>
 * </xsd:complexType>
 *
 * ========================================================================== */
struct AbsoluteTargetSpeed
{
  const Double value;

  template<typename Node, typename Scope>
  explicit AbsoluteTargetSpeed(const Node & node, Scope & scope)
  : value{readAttribute<Double>("value", node, scope)}
  {}
};
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ABSOLUTE_TARGET_SPEED_HPP_
