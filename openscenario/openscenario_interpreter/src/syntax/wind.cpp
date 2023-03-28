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
#include <openscenario_interpreter/syntax/wind.hpp>
#include <scenario_simulator_exception/exception.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Wind::Wind(const pugi::xml_node & node, Scope & scope)
: direction(readAttribute<Double>("direction", node, scope)),
  speed(readAttribute<Double>("speed", node, scope))
{
  auto direction_valid = 0 <= direction and direction <= 2 * boost::math::constants::pi<double>();
  if (!direction_valid) {
    THROW_SYNTAX_ERROR(std::quoted("direction"), "is out of range [0...2 pi[");
  }
  auto speed_valid = 0 <= speed;
  if (!speed_valid) {
    THROW_SYNTAX_ERROR(std::quoted("speed"), "is out of range [0...inf[");
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
