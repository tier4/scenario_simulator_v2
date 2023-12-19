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
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/relative_target_speed.hpp>
#include <openscenario_interpreter/syntax/rule.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
RelativeTargetSpeed::RelativeTargetSpeed(const pugi::xml_node & node, Scope & scope)
: entity_ref(readAttribute<String>("entityRef", node, scope), scope),
  value(readAttribute<Double>("value", node, scope)),
  speed_target_value_type(readAttribute<SpeedTargetValueType>(
    "speedTargetValueType", node, scope, SpeedTargetValueType())),
  continuous(readAttribute<Boolean>("continuous", node, scope, Boolean()))
{
}
}  // namespace syntax
}  // namespace openscenario_interpreter
