// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <openscenario_interpreter/syntax/absolute_target_speed.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto AbsoluteTargetSpeed::getIsEnd() const -> std::function<bool(const EntityRef &)>
{
  return [target_speed = value](const EntityRef & actor) {  // is_end
    try {
      const auto compare = Rule(Rule::equalTo);
      return compare(getEntityStatus(actor).action_status.twist.linear.x, target_speed);
    } catch (const SemanticError &) {
      return false;  // NOTE: The actor is maybe lane-changing now
    }
  };
}
}  // namespace syntax
}  // namespace openscenario_interpreter
