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

#include <boost/math/constants/constants.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/sun.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Sun::Sun(const pugi::xml_node & node, Scope & scope)
: azimuth(readAttribute<Double>("azimuth", node, scope)),
  elevation(readAttribute<Double>("elevation", node, scope)),
  illuminance(readAttribute<Double>("illuminance", node, scope, 0)),
  intensity(readAttribute<Double>("intensity", node, scope))
{
  // Valid range ref:
  // https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/content/Sun.html

  if (auto azimuth_valid = 0 <= azimuth and azimuth <= 2 * boost::math::constants::pi<double>();
      not azimuth_valid) {
    THROW_SYNTAX_ERROR(std::quoted("Sun::azimuth"), "is out of range [0..2*PI]");
  }

  if (auto elevation_valid = -boost::math::constants::pi<double>() <= elevation and
                             elevation <= boost::math::constants::pi<double>();
      not elevation_valid) {
    THROW_SYNTAX_ERROR(std::quoted("Sun::elevation"), "is out of range [-PI..PI]");
  }

  if (auto illuminance_valid = 0 <= illuminance; not illuminance_valid) {
    THROW_SYNTAX_ERROR(std::quoted("Sun::illuminance"), "is out of range [0..inf[");
  }

  if (auto intensity_valid = 0 <= intensity; not intensity_valid) {
    THROW_SYNTAX_ERROR(std::quoted("Sun::intensity"), "is out of range [0..inf[");
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
