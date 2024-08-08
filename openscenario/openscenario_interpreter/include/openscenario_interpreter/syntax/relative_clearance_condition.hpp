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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_CLEARANCE_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_CLEARANCE_CONDITION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/relative_lane_range.hpp>
#include <openscenario_interpreter/syntax/routing_algorithm.hpp>
#include <openscenario_interpreter/syntax/triggering_entities.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   RelativeClearanceCondition (OpenSCENARIO XML 1.3)

   <xsd:complexType name="RelativeClearanceCondition">
     <xsd:sequence>
       <xsd:element name="RelativeLaneRange" type="RelativeLaneRange" minOccurs="0" maxOccurs="unbounded"/>
       <xsd:element name="EntityRef" type="EntityRef" minOccurs="0" maxOccurs="unbounded"/>
     </xsd:sequence>
     <xsd:attribute name="oppositeLanes" type="Boolean" use="required"/>
     <xsd:attribute name="distanceForward" type="Double"/>
     <xsd:attribute name="distanceBackward" type="Double"/>
     <xsd:attribute name="freeSpace" type="Boolean" use="required"/>
   </xsd:complexType>
*/
struct RelativeClearanceCondition : private Scope,
                                    private SimulatorCore::ConditionEvaluation,
                                    private SimulatorCore::NonStandardOperation
{
  /*
     Longitudinal distance behind reference point of the entity to be checked along lane centerline of the current lane of the triggering entity. Orientation of entity towards lane determines backward direction. Velocity of entity is irrelevant. Unit: [m]. Range: [0..inf[. Default if omitted: 0
  */
  const Double distance_backward;

  /*
     Longitudinal distance in front of reference point of the entity to be checked along lane centerline of the current lane of the triggering entity. Orientation of entity towards lane determines forward direction. Velocity of entity is irrelevant. Unit: [m]. Range: [0..inf[. Default if omitted: 0
  */
  const Double distance_forward;

  /*
     If false, then entityRefs are only considered to be on the lane if their reference point is within the checked area; otherwise the whole bounding box is considered.
  */
  const Boolean free_space;

  /*
     If true, then also lanes in the opposite direction are considered; otherwise only lanes in the same direction are considered.
  */
  const Boolean opposite_lanes;

  /*
     The lanes to be checked to left and right of the triggering entity (positive to the y-axis). If omitted: all lanes are checked.
  */
  const std::list<RelativeLaneRange> relative_lane_range;

  /*
     Constraint to check only specific entities. If it is not used then all entities are considered.
  */
  const std::list<Entity> entity_refs;

  const TriggeringEntities triggering_entities;

  explicit RelativeClearanceCondition(const pugi::xml_node &, Scope &, const TriggeringEntities &);

  auto description() const -> String;

  auto evaluate() -> Object;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_CLEARANCE_CONDITION_HPP_
