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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__WIND_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__WIND_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Wind 1.2 -----------------------------------------------------------
 *
 * <xsd:complexType name="Wind">
 *   <xsd:attribute name="direction" type="Double" use="required"/>
 *   <xsd:attribute name="speed" type="Double" use="required"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Wind
{
  const Double direction;  // The target direction of the wind (not the origin direction) in the
    // ground/xy-plane of the world coordinate system. Corresponds to the heading/yaw
    // angle. x-axis-direction is 0 rad. Unit: [rad]. Range: [0..2*PI[.

  const Double speed;  // The wind speed. Unit: [m/s]. Range: [0..inf[

  Wind() = default;

  explicit Wind(const pugi::xml_node &, Scope &);
};

}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__WIND_HPP_
