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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ROUTING_ALGORITHM_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ROUTING_ALGORITHM_HPP_

#include <iostream>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   RoutingAlgorithm (OpenSCENARIO XML 1.3)

   Algorithm for path selection/calculation between two positions across roads.
   To be used for distance calculations in road/lane coordinates during runtime.

   <xsd:simpleType name="RoutingAlgorithm">
     <xsd:union>
       <xsd:simpleType>
         <xsd:restriction base="xsd:string">
           <xsd:enumeration value="assignedRoute"/>
           <xsd:enumeration value="fastest"/>
           <xsd:enumeration value="leastIntersections"/>
           <xsd:enumeration value="shortest"/>
           <xsd:enumeration value="undefined"/>
         </xsd:restriction>
       </xsd:simpleType>
       <xsd:simpleType>
         <xsd:restriction base="parameter"/>
       </xsd:simpleType>
     </xsd:union>
   </xsd:simpleType>
*/
struct RoutingAlgorithm
{
  enum value_type {
    /*
       It is up to the simulator how to calculate the route between the start
       and target positions.
    */
    undefined,  // NOTE: DEFAULT VALUE

    /*
       Use the route which has already been assigned to the entity at the start
       position at the point in time when the distance shall be calculated.
    */
    assigned_route,

    /*
       Calculate the route with the shortest travelling time between start and
       target position.
    */
    fastest,

    /*
       Calculate the route with as few junctions as possible between start and
       target position.
    */
    least_intersections,

    /*
       Calculate the route with the shortest path between start and target
       position.
    */
    shortest,
  } value;

  RoutingAlgorithm() = default;

  constexpr RoutingAlgorithm(value_type value) : value(value) {}

  constexpr operator value_type() const noexcept { return value; }
};

auto operator>>(std::istream &, RoutingAlgorithm &) -> std::istream &;

auto operator<<(std::ostream &, const RoutingAlgorithm &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ROUTING_ALGORITHM_HPP_
