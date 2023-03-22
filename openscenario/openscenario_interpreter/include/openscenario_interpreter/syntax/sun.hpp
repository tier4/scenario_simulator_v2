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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__SUN_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__SUN_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Sum -------------------------------------------------------------
 *
 * <xsd:complexType name="Sun">
 *   <xsd:attribute name="azimuth" type="Double" use="required"/>
 *   <xsd:attribute name="elevation" type="Double" use="required"/>
 *   <xsd:attribute name="intensity" type="Double">
 *    <xsd:annotation>
 *      <xsd:appinfo>
 *        deprecated
 *      </xsd:appinfo>
 *    </xsd:annotation>
 *   </xsd:attribute>
 *   <xsd:attribute name="illuminance" type="Double"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Sun
{
  const Double azimuth;  // Azimuth of the sun, counted clockwise, 0=north, PI/2 = east, PI=south,
                         // 3/2 PI=west. Unit: [rad]. Range: [0..2*PI].

  const Double
    elevation;  // Solar elevation angle, 0=x/y plane, PI/2=zenith. Unit: [rad]. Range: [-PI..PI].

  const Double illuminance;  //	Illuminance of the sun, direct sunlight is around 100,000 lx. Unit:
                             //[lx]. Range: [0..inf[. Default if missing: 0.

  const Double intensity;  // DEPRECATED: Illuminance of the sun, direct sunlight is around 100,000
                           // lx. Unit: [lx]. Range: [0..inf[.

  explicit Sun(const pugi::xml_node &, Scope &);
};

}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__SUN_HPP_
