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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__FOLLOW_TRAJECTORY_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__FOLLOW_TRAJECTORY_ACTION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/entity.hpp>
#include <openscenario_interpreter/syntax/time_reference.hpp>
#include <openscenario_interpreter/syntax/trajectory_following_mode.hpp>
#include <openscenario_interpreter/syntax/trajectory_ref.hpp>
#include <pugixml.hpp>
#include <traffic_simulator/behavior/follow_trajectory.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- FollowTrajectoryAction 1.2 ---------------------------------------------
 *
 *  <xsd:complexType name="FollowTrajectoryAction">
 *    <xsd:all>
 *      <xsd:element name="Trajectory" type="Trajectory" minOccurs="0">
 *        <xsd:annotation>
 *          <xsd:appinfo>
 *            deprecated
 *          </xsd:appinfo>
 *        </xsd:annotation>
 *      </xsd:element>
 *      <xsd:element name="CatalogReference" type="CatalogReference" minOccurs="0">
 *        <xsd:annotation>
 *          <xsd:appinfo>
 *            deprecated
 *          </xsd:appinfo>
 *        </xsd:annotation>
 *      </xsd:element>
 *      <xsd:element name="TimeReference" type="TimeReference"/>
 *      <xsd:element name="TrajectoryFollowingMode" type="TrajectoryFollowingMode"/>
 *      <xsd:element name="TrajectoryRef" type="TrajectoryRef" minOccurs="0"/>
 *    </xsd:all>
 *    <xsd:attribute name="initialDistanceOffset" type="Double"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct FollowTrajectoryAction : private Scope,
                                private SimulatorCore::ActionApplication,
                                private SimulatorCore::ConditionEvaluation,
                                private SimulatorCore::NonStandardOperation
{
  const Double initial_distance_offset;

  const TimeReference time_reference;

  const TrajectoryFollowingMode trajectory_following_mode;

  const TrajectoryRef trajectory_ref;

  std::unordered_map<Entity, Boolean> accomplishments;

  explicit FollowTrajectoryAction(const pugi::xml_node &, Scope &);

  /*  */ auto accomplished() -> bool;

  static auto endsImmediately() noexcept -> bool;

  static auto run() -> void;

  /*  */ auto start() -> void;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__FOLLOW_TRAJECTORY_ACTION_HPP_
