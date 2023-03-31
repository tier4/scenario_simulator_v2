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
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/road_condition.hpp>
#include <openscenario_interpreter/syntax/wetness.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
RoadCondition::RoadCondition(const pugi::xml_node & node, Scope & scope)
: friction_scale_factor(readAttribute<Double>("frictionScaleFactor", node, scope)),
  wetness(readAttribute<Wetness>("wetness", node, scope)),
  properties(readElement<Properties>("Properties", node, scope))
{
  // Valid range ref:
  // https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/content/RoadCondition.html
  auto friction_scale_factor_valid = 0 <= friction_scale_factor;
  if (!friction_scale_factor_valid) {
    THROW_SYNTAX_ERROR(
      std::quoted("RoadCondition::frictionScaleFactor"), "is out of range [0..inf[");
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
