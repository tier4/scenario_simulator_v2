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

#include <iomanip>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/weather.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Weather::Weather(const pugi::xml_node & node, Scope & scope)
: atmospheric_pressure(readAttribute<std::optional<Double>>("atmosphericPressure", node, scope)),
  temperature(readAttribute<std::optional<Double>>("temperature", node, scope)),
  fractional_cloud_cover(
    readAttribute<std::optional<FractionalCloudCover>>("fractionalCloudCover", node, scope)),
  sun(readElement<std::optional<Sun>>("Sun", node, scope)),
  fog(readElement<std::optional<Fog>>("Fog", node, scope)),
  precipitation(readElement<std::optional<Precipitation>>("Precipitation", node, scope)),
  wind(readElement<std::optional<Wind>>("Wind", node, scope)),
  dome_image(readElement<std::optional<DomeImage>>("DomeImage", node, scope))
{
  // Valid range ref:
  // https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/content/Weather.html

  // [80000...120000]
  if (auto atmospheric_pressure_valid =
        80000 <= atmospheric_pressure and atmospheric_pressure <= 120000;
      not atmospheric_pressure_valid) {
    THROW_SYNTAX_ERROR(
      std::quoted("Weather::atmosphericPressure"), "is out of range [80000...120000]");
  }

  // [170...340]
  if (auto temperature_valid = 170 <= temperature and temperature <= 340; not temperature_valid) {
    THROW_SYNTAX_ERROR(std::quoted("Weather::temperature"), "is out of range [170...340]");
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
