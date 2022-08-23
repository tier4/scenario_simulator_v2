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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__LATERAL_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__LATERAL_ACTION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/lane_change_action.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- LateralAction ----------------------------------------------------------
 *
 *  <xsd:complexType name="LateralAction">
 *    <xsd:choice>
 *      <xsd:element name="LaneChangeAction" type="LaneChangeAction"/>
 *      <xsd:element name="LaneOffsetAction" type="LaneOffsetAction"/>
 *      <xsd:element name="LateralDistanceAction" type="LateralDistanceAction"/>
 *    </xsd:choice>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct LateralAction : public ComplexType
{
  explicit LateralAction(const pugi::xml_node &, Scope &);

  auto endsImmediately() const -> bool;

  auto run() -> void;

  auto start() -> void;
};

DEFINE_LAZY_VISITOR(
  LateralAction,
  CASE(LaneChangeAction),  //
  // CASE(LaneOffsetAction),
  // CASE(LateralDistanceAction),
);

DEFINE_LAZY_VISITOR(
  const LateralAction,
  CASE(LaneChangeAction),  //
  // CASE(LaneOffsetAction),
  // CASE(LateralDistanceAction),
);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__LATERAL_ACTION_HPP_
