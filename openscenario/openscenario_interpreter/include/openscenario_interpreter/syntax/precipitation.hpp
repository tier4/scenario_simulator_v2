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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PRECIPITATION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PRECIPITATION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/bounding_box.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/precipitation_type.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Precipitation 1.2 ----------------------------------------------------
 *
 * <xsd:complexType name="Precipitation">
 *   <xsd:attribute name="intensity" type="Double">
 *     <xsd:annotation>
 *       <xsd:appinfo>
 *         deprecated
 *       </xsd:appinfo>
 *     </xsd:annotation>
 *   </xsd:attribute>
 *   <xsd:attribute name="precipitationType" type="PrecipitationType" use="required"/>
 *   <xsd:attribute name="precipitationIntensity" type="Double"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Precipitation
{
  const Double intensity;  // DEPRECATED: The intensity of the precipitation. Range: [0..1].

  const Double precipitation_intensity;  // The intensity of the precipitation (valid for all
                                         // precipitation types). Unit: [mm/h]. Range: [0..inf[.

  const PrecipitationType precipitation_type;

  Precipitation();
  explicit Precipitation(const pugi::xml_node &, Scope &);
};

}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PRECIPITATION_HPP_
