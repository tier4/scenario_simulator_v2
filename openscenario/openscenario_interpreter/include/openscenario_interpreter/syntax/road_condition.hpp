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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ROAD_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ROAD_CONDITION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/properties.hpp>
#include <openscenario_interpreter/syntax/wetness.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- RoadCondition 1.2 ---------------------------------------------------
 *
 * <xsd:complexType name="RoadCondition">
 *   <xsd:sequence>
 *     <xsd:element name="Properties" type="Properties" minOccurs="0"/>
 *   </xsd:sequence>
 *   <xsd:attribute name="frictionScaleFactor" type="Double" use="required"/>
 *   <xsd:attribute name="wetness" type="Wetness"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct RoadCondition
{
  const Double friction_scale_factor;  // 	Friction scale factor. Range: [0..inf[.

  const Wetness wetness;  //	Definition of the wetness of the road.

  const Properties properties;  // Additional properties to describe the road condition.

  RoadCondition() = default;
  explicit RoadCondition(const pugi::xml_node &, Scope &);
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ROAD_CONDITION_HPP_
