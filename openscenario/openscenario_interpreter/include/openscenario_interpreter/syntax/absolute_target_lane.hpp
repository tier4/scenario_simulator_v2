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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ABSOLUTE_TARGET_LANE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ABSOLUTE_TARGET_LANE_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
  AbsoluteTargetLane (OpenSCENARIO XML 1.3.0)

  Defines the number (ID) of the target lane.

  <xsd:complexType name="AbsoluteTargetLane">
    <xsd:attribute name="value" type="String" use="required"/>
  </xsd:complexType>
*/
struct AbsoluteTargetLane
{
  const String value;

  explicit AbsoluteTargetLane(const pugi::xml_node &, Scope &);
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ABSOLUTE_TARGET_LANE_HPP_
