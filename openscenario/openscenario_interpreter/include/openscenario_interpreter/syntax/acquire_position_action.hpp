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

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/position.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/utility/overload.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <unordered_map>

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

  template <typename Node>
  explicit AcquirePositionAction(const Node & node, Scope & outer_scope)
  : Scope(outer_scope), position(readElement<Position>("Position", node, localScope()))
  {
  }

  static constexpr auto accomplished() -> bool { return true; }

  static constexpr auto endsImmediately() -> bool { return true; };

  auto start() -> Element
  {
    const auto acquire_position = overload(
      [](const WorldPosition & position, auto && actor) {
        return applyAcquirePositionAction(actor, static_cast<geometry_msgs::msg::Pose>(position));
      },
      [](const RelativeWorldPosition & position, auto && actor) {
        return applyAcquirePositionAction(
          actor, static_cast<openscenario_msgs::msg::LaneletPose>(position));
      },
      [](const LanePosition & position, auto && actor) {
        return applyAcquirePositionAction(
          actor, static_cast<openscenario_msgs::msg::LaneletPose>(position));
      });

    for (const auto & actor : actors) {
      apply(acquire_position, position, actor);
    }

    return unspecified;
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ACQUIRE_POSITION_ACTION_HPP_
