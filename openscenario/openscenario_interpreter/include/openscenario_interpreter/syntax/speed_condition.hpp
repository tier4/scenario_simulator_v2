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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_CONDITION_HPP_

#include <openscenario_interpreter/compatibility.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/directional_dimension.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/rule.hpp>
#include <openscenario_interpreter/syntax/triggering_entities.hpp>
#include <pugixml.hpp>
#include <valarray>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   SpeedCondition (OpenSCENARIO XML 1.3.1)

   Compares a triggering entity's/entities' speed to a target speed. The
   logical operator for the comparison is given by the rule attribute. If
   direction is used, only the projection to that direction is used in the
   comparison.

   <xsd:complexType name="SpeedCondition">
     <xsd:attribute name="rule" type="Rule" use="required"/>
     <xsd:attribute name="value" type="Double" use="required"/>
     <xsd:attribute name="direction" type="DirectionalDimension"/>
   </xsd:complexType>
*/
struct SpeedCondition : private Scope, private SimulatorCore::ConditionEvaluation
{
  /*
     The operator (less, greater, equal).
  */
  const Rule rule;

  /*
     Speed value of the speed condition. Unit: [m/s].
  */
  const Double value;

  /*
     Direction of the speed (if not given, the total speed is considered).
  */
  const std::optional<DirectionalDimension> direction;

  const TriggeringEntities triggering_entities;

  std::vector<std::valarray<double>> results;  // for description

  static inline auto compatibility = Compatibility::legacy;

  explicit SpeedCondition(const pugi::xml_node &, Scope &, const TriggeringEntities &);

  auto description() const -> String;

  static auto evaluate(const Entities *, const Entity &) -> Eigen::Vector3d;

  static auto evaluate(
    const Entities *, const Entity &, const std::optional<DirectionalDimension> &,
    const Compatibility = Compatibility::legacy) -> double;

  auto evaluate() -> Object;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_CONDITION_HPP_
