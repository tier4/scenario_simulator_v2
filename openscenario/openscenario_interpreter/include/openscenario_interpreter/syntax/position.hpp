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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__POSITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__POSITION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/lane_position.hpp>
#include <openscenario_interpreter/syntax/relative_object_position.hpp>
#include <openscenario_interpreter/syntax/relative_world_position.hpp>
#include <openscenario_interpreter/syntax/world_position.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   Position (OpenSCENARIO XML 1.3)

   <xsd:complexType name="Position">
     <xsd:choice>
       <xsd:element name="WorldPosition" type="WorldPosition"/>
       <xsd:element name="RelativeWorldPosition" type="RelativeWorldPosition"/>
       <xsd:element name="RelativeObjectPosition" type="RelativeObjectPosition"/>
       <xsd:element name="RoadPosition" type="RoadPosition"/>
       <xsd:element name="RelativeRoadPosition" type="RelativeRoadPosition"/>
       <xsd:element name="LanePosition" type="LanePosition"/>
       <xsd:element name="RelativeLanePosition" type="RelativeLanePosition"/>
       <xsd:element name="RoutePosition" type="RoutePosition"/>
       <xsd:element name="GeoPosition" type="GeoPosition"/>
       <xsd:element name="TrajectoryPosition" type="TrajectoryPosition"/>
     </xsd:choice>
   </xsd:complexType>
*/
struct Position : public ComplexType
{
  explicit Position(const pugi::xml_node &, Scope &);

  explicit operator geometry_msgs::msg::Pose() const;

  explicit operator NativeLanePosition() const;
};

DEFINE_LAZY_VISITOR(
  Position,
  CASE(WorldPosition),           //
  CASE(RelativeWorldPosition),   //
  CASE(RelativeObjectPosition),  //
  // CASE(RoadPosition),
  // CASE(RelativeRoadPosition),
  CASE(LanePosition),
  // CASE(RelativeLanePosition),
  // CASE(RoutePosition),
  // CASE(GeoPosition),
  // CASE(TrajectoryPosition),
);

DEFINE_LAZY_VISITOR(
  const Position,
  CASE(WorldPosition),           //
  CASE(RelativeWorldPosition),   //
  CASE(RelativeObjectPosition),  //
  // CASE(RoadPosition),
  // CASE(RelativeRoadPosition),
  CASE(LanePosition),
  // CASE(RelativeLanePosition),
  // CASE(RoutePosition),
  // CASE(GeoPosition),
  // CASE(TrajectoryPosition),
);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__POSITION_HPP_
