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
#include <openscenario_interpreter/syntax/speed_profile_action.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
SpeedProfileAction::SpeedProfileAction(const pugi::xml_node & node, Scope & scope)
: entity_ref(readAttribute<EntityRef>("entityRef", node, scope)),
  following_mode(readAttribute<FollowingMode>("followingMode", node, scope)),
  dynamic_constraints(readElement<DynamicConstraints>("DynamicConstraints", node, scope)),
  speed_profile_entry(readElements<SpeedProfileEntry, 1>("SpeedProfileEntry", node, scope))
{
}

auto SpeedProfileAction::accomplished() -> bool
{
  return true;  // TODO
}

auto SpeedProfileAction::endsImmediately() const -> bool
{
  return true;  // TODO
}

auto SpeedProfileAction::run() -> void {}

auto SpeedProfileAction::start() -> void {}
}  // namespace syntax
}  // namespace openscenario_interpreter
