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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/syntax/precipitation.hpp>
#include <openscenario_interpreter/syntax/precipitation_type.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Precipitation::Precipitation(const pugi::xml_node & node, Scope & scope)
: intensity(readAttribute<Double>("intensity", node, scope, 0)),
  precipitation_intensity(readAttribute<Double>("precipitationIntensity", node, scope, 0)),
  precipitation_type(readAttribute<PrecipitationType>("precipitationType", node, scope))
{
  // Valid range ref:
  // https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/content/Precipitation.html

  if (auto intensity_valid = 0 <= intensity and intensity <= 1; not intensity_valid) {
    THROW_SYNTAX_ERROR(std::quoted("Precipitation::intensity"), "is out of range [0..1]");
  }

  if (auto precipitation_intensity_valid = 0 <= intensity; not precipitation_intensity_valid) {
    THROW_SYNTAX_ERROR(
      std::quoted("Precipitation::precipitationIntensity"), "is out of range [0..inf[");
  }
}

}  // namespace syntax
}  // namespace openscenario_interpreter
