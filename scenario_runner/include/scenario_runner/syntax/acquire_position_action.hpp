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

#ifndef SCENARIO_RUNNER__SYNTAX__ACQUIRE_POSITION_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__ACQUIRE_POSITION_ACTION_HPP_

#include <scenario_runner/syntax/position.hpp>

#include <string>
#include <unordered_map>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== AcquirePositionAction ================================================
 *
 * <xsd:complexType name="AcquirePositionAction">
 *   <xsd:all>
 *     <xsd:element name="Position" type="Position"/>
 *   </xsd:all>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct AcquirePositionAction
{
  Scope inner_scope;

  const Position position;

  template<typename Node>
  explicit AcquirePositionAction(const Node & node, Scope & outer_scope)
  : inner_scope{outer_scope},
    position{readElement<Position>("Position", node, inner_scope)}
  {}

  std::unordered_map<std::string, Boolean> accomplishments;

  auto start()
  {
    accomplishments.clear();

    if (position.is<LanePosition>()) {
      for (const auto & actor : inner_scope.actors) {
        accomplishments.emplace(actor, false);

        // inner_scope.connection->entity->requestAcquirePosition(
        //   actor,
        //   Integer(position.as<LanePosition>().lane_id),
        //   position.as<LanePosition>().s,
        //   position.as<LanePosition>().offset);
      }
    } else {
      THROW(ImplementationFault);
    }
  }

  auto accomplished()
  {
    #ifndef SCENARIO_RUNNER_NO_EXTENSION
    if (position.is<LanePosition>()) {
      for (auto && each : accomplishments) {
        if (!cdr(each)) {
          // cdr(each) =
          //   inner_scope.connection->entity->reachPosition(
          //     car(each),
          //     Integer(position.as<LanePosition>().lane_id),
          //     position.as<LanePosition>().s,
          //     position.as<LanePosition>().offset,
          //     5.0);
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
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__ACQUIRE_POSITION_ACTION_HPP_
