// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__ACQUIRE_POSITION_ACTION_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__ACQUIRE_POSITION_ACTION_HPP_

#include <open_scenario_interpreter/procedure.hpp>
#include <open_scenario_interpreter/syntax/position.hpp>

#include <string>
#include <unordered_map>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ---- AcquirePositionAction --------------------------------------------------
 *
 * <xsd:complexType name="AcquirePositionAction">
 *   <xsd:all>
 *     <xsd:element name="Position" type="Position"/>
 *   </xsd:all>
 * </xsd:complexType>
 *
 * TODO REMOVE EXTENSION
 *
 * -------------------------------------------------------------------------- */
struct AcquirePositionAction
{
  Scope inner_scope;

  const Position position;

  template<typename Node>
  explicit AcquirePositionAction(const Node & node, Scope & outer_scope)
  : inner_scope(outer_scope),
    position(readElement<Position>("Position", node, inner_scope))
  {}

  std::unordered_map<std::string, Boolean> accomplishments;

  auto start()
  {
    accomplishments.clear();

    if (position.is<LanePosition>()) {
      for (const auto & actor : inner_scope.actors) {
        accomplishments.emplace(actor, false);

        requestAcquirePosition(
          actor,
          Integer(position.as<LanePosition>().lane_id),
          position.as<LanePosition>().s,
          position.as<LanePosition>().offset);
      }
    } else {
      THROW(ImplementationFault);
    }
  }

  auto accomplished()
  {
    #ifndef OPEN_SCENARIO_INTERPRETER_NO_EXTENSION
    if (position.is<LanePosition>()) {
      for (auto && each : accomplishments) {
        if (!cdr(each)) {
          cdr(each) = isReachedPosition(
            car(each),
            Integer(position.as<LanePosition>().lane_id),
            position.as<LanePosition>().s,
            position.as<LanePosition>().offset,
            5.0);
        }
      }
      return std::all_of(std::begin(accomplishments), std::end(accomplishments), cdr);
    } else {
      THROW(ImplementationFault);
    }
    #else
    return true;
    #endif
  }
};
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__ACQUIRE_POSITION_ACTION_HPP_
