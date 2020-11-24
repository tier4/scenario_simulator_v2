// Copyright 2015-2020 TierIV.inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__STAND_STILL_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__STAND_STILL_CONDITION_HPP_

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/rule.hpp>
#include <openscenario_interpreter/syntax/triggering_entities.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- StandStillCondition ---------------------------------------------------------
 *
 * <xsd:complexType name="StandStillCondition">
 *   <xsd:attribute name="duration" type="Double" use="required"/>
 * </xsd:complexType>*
 *
 * -------------------------------------------------------------------------- */
struct StandStillCondition
{
  const Double duration;

  const Rule compare;

  const TriggeringEntities for_each;

  template<typename Node>
  explicit StandStillCondition(
    const Node & node, Scope & outer_scope, const TriggeringEntities & for_each)
  : duration(readAttribute<Double>("duration", node, outer_scope)),
    compare(Rule::greaterThan),
    for_each(for_each)
  {}

  auto evaluate() const
  {
    return asBoolean(
      for_each([&](auto && triggering_entity)
      {
        return compare(getStandStillDuration(triggering_entity), duration);
      }));
  }
};
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__STAND_STILL_CONDITION_HPP_
