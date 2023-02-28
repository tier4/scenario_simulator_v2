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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__LANE_POSITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__LANE_POSITION_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/orientation.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- LanePosition -----------------------------------------------------------
 *
 *  <xsd:complexType name="LanePosition">
 *    <xsd:all>
 *      <xsd:element name="Orientation" type="Orientation" minOccurs="0"/>
 *    </xsd:all>
 *    <xsd:attribute name="roadId" type="String" use="required"/>
 *    <xsd:attribute name="laneId" type="String" use="required"/>
 *    <xsd:attribute name="offset" type="Double" use="optional"/>
 *    <xsd:attribute name="s" type="Double" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct LanePosition : private SimulatorCore::CoordinateSystemConversion
{
  const String road_id, lane_id;

  const Double offset, s;

  const Orientation orientation;

  explicit LanePosition(const pugi::xml_node &, Scope &);

  explicit LanePosition(
    const String &, const String &, const Double &, const Double &, const Orientation & = {});

  explicit operator NativeLanePosition() const;

  explicit operator CanonicalizedLanePosition() const;

  explicit operator NativeWorldPosition() const;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__LANE_POSITION_HPP_
