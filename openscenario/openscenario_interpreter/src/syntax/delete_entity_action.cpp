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

#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/delete_entity_action.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto DeleteEntityAction::operator()(const EntityRef & entity_ref) const -> void
{
  applyDeleteEntityAction(entity_ref);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
