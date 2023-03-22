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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__FOG_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__FOG_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/bounding_box.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Fog 1.2 ------------------------------------------------------------
 *
 * <xsd:complexType name="Fog">
 *   <xsd:all>
 *     <xsd:element name="BoundingBox" type="BoundingBox" minOccurs="0"/>
 *   </xsd:all>
 *   <xsd:attribute name="visualRange" type="Double" use="required"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Fog
{
  const Double visual_range;  // Unit: [m]. Range: [0..inf[.

  const BoundingBox bounding_box;  // Dimensions and center of fog in fixed coordinates.

  Fog() = default;
  explicit Fog(const pugi::xml_node &, Scope &);
};

}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__FOG_HPP_
