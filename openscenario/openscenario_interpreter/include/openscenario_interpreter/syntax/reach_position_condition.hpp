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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__REACH_POSITION_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__REACH_POSITION_CONDITION_HPP_

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/position.hpp>
#include <openscenario_interpreter/syntax/triggering_entities.hpp>

#include <simulation_api/helper/helper.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ReachPositionCondition -------------------------------------------------
 *
 *  <xsd:complexType name="ReachPositionCondition">
 *    <xsd:all>
 *      <xsd:element name="Position" type="Position"/>
 *    </xsd:all>
 *    <xsd:attribute name="tolerance" type="Double" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ReachPositionCondition
{
  const Double tolerance;

  const Position position;

  template<typename Node>
  explicit ReachPositionCondition(
    const Node & node, Scope & outer_scope, const TriggeringEntities & triggering_entities)
  : tolerance(readAttribute<Double>("tolerance", node, outer_scope)),
    position(readElement<Position>("Position", node, outer_scope)),
    for_each(triggering_entities)
  {}

  const TriggeringEntities for_each;

  auto check(const TriggeringEntities::value_type & name)
  {
    if (position.is<WorldPosition>()) {
      return isReachedPosition(name, position.as<WorldPosition>(), tolerance);
    } else if (position.is<LanePosition>()) {
      return isReachedPosition(
        name,
        static_cast<openscenario_msgs::msg::LaneletPose>(position.as<LanePosition>()),
        tolerance);
    } else {
      THROW(ImplementationFault);
    }
  }

  auto evaluate()
  {
    #ifndef NDEBUG
    std::cout << (indent++) << "- BEC.RPC:\n";
    #endif

    const auto result = asBoolean(
      for_each(
        [&](auto && triggering_entity)
        {
          const auto result = check(triggering_entity);
          #ifndef NDEBUG
          std::cout << indent << "  " << triggering_entity << ": ";
          std::cout << std::boolalpha << result;
          std::cout << " (tolerance = " << tolerance << ")" << std::endl;
          #endif
          return result;
        }));

    #ifndef NDEBUG
    --indent;
    #endif

    return result;
  }
};
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__REACH_POSITION_CONDITION_HPP_
