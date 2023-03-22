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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__WEATHER_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__WEATHER_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/dome_image.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/fog.hpp>
#include <openscenario_interpreter/syntax/fractional_cloud_cover.hpp>
#include <openscenario_interpreter/syntax/precipitation.hpp>
#include <openscenario_interpreter/syntax/sun.hpp>
#include <openscenario_interpreter/syntax/wind.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Weather 1.2 ---------------------------------------------------
 *
 * <xsd:complexType name="Weather">
 * 　　<xsd:all>
 * 　　　　<xsd:element name="Sun" type="Sun" minOccurs="0"/>
 * 　　　　<xsd:element name="Fog" type="Fog" minOccurs="0"/>
 * 　　　　<xsd:element name="Precipitation" type="Precipitation" minOccurs="0"/>
 * 　　　　<xsd:element name="Wind" type="Wind" minOccurs="0"/>
 * 　　　　<xsd:element name="DomeImage" type="DomeImage" minOccurs="0"/>
 * 　　</xsd:all>
 * 　　<xsd:attribute name="cloudState" type="CloudState">
 * 　　　<xsd:annotation>
 * 　　　　　<xsd:appinfo>
 * 　　　　　　　deprecated
 * 　　　　　</xsd:appinfo>
 * 　　　</xsd:annotation>
 * 　　</xsd:attribute>
 * 　　<xsd:attribute name="atmosphericPressure" type="Double"/>
 * 　　<xsd:attribute name="temperature" type="Double"/>
 * 　　<xsd:attribute name="fractionalCloudCover" type="FractionalCloudCover"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Weather
{
  // TODO: To realize "If one of the conditions is missing it means that it doesn't change.", we
  // should use "optional" rather than default constructor of each element.

  const Double atmospheric_pressure;  // Reference atmospheric pressure at z=0.0 in world coordinate
                                      // system. Unit: [Pa]. Range: [80000..120000]. The actual
                                      // atmospheric pressure around the entities of the scenario
                                      // has to be calculated depending on their z position. See
                                      // also the Standard Atmosphere as defined in ISO2533.

  const Double
    temperature;  // Definition of the cloud state, i.e. cloud state and sky visualization settings.

  const FractionalCloudCover fractional_cloud_cover;  //	Definition of cloud states using the
                                                      // fractional cloud cover in oktas.
  const Sun sun;  // Definition of the sun, i.e. position and intensity.

  const Fog fog;  //	Definition of fog, i.e. visual range and bounding box.

  const Precipitation precipitation;  //	Definition of precipitation, i.e. type and
  // intensity.

  const Wind wind;  // 	Definition of the wind: direction and speed.

  const DomeImage dome_image;  // 	Image reference to represent the sky. Mutually exclusive
                               // with "fractionalCloudCover". If the image also contains lighting
                               // information (HDRi) it is also mutually exclusive with "sun".

  explicit Weather(const pugi::xml_node &, Scope &);
};

}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__WEATHER_HPP_
