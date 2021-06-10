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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TELEPORT_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TELEPORT_ACTION_HPP_

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/position.hpp>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- TeleportAction ---------------------------------------------------------
 *
 * <xsd:complexType name="TeleportAction">
 *   <xsd:sequence>
 *     <xsd:element name="Position" type="Position"/>
 *   </xsd:sequence>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct TeleportAction
{
  Scope inner_scope;

  const Position position;

  template <typename Node>
  explicit TeleportAction(const Node & node, Scope & outer_scope)
  : inner_scope(outer_scope), position(readElement<Position>("Position", node, inner_scope))
  {
  }

  const std::true_type accomplished{};

  decltype(auto) operator()(const WorldPosition & world_position, const Scope::Actor & actor) const
  {
    return setEntityStatus(actor, static_cast<geometry_msgs::msg::Pose>(world_position));
  }

  decltype(auto) operator()(const LanePosition & lane_position, const Scope::Actor & actor) const
  {
    return setEntityStatus(actor, static_cast<openscenario_msgs::msg::LaneletPose>(lane_position));
  }

  decltype(auto) operator()(
    const RelativeWorldPosition & relative_world_position, const Scope::Actor & actor) const
  {
    return setEntityStatus(
      actor,
      relative_world_position.reference,  // name
      relative_world_position,            // geometry_msgs::msg::Point
      relative_world_position.orientation);
  }

  void start() const
  {
    for (const auto & actor : inner_scope.actors) {
      apply(*this, position, actor);
    }
  }

  const std::true_type is_complete_immediately{};
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TELEPORT_ACTION_HPP_
