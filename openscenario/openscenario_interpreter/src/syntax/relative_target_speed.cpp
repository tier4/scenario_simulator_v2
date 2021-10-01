// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/relative_target_speed.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto RelativeTargetSpeed::getCalculateAbsoluteTargetSpeed() const -> std::function<double()>
{
  if (speed_target_value_type == SpeedTargetValueType::factor) {
    return [factor = value, entity_ref = entity_ref]() -> double {
      return factor * getEntityStatus(entity_ref).action_status.twist.linear.x;
    };
  } else if (speed_target_value_type == SpeedTargetValueType::delta) {
    return [delta = value, entity_ref = entity_ref]() -> double {
      return delta + getEntityStatus(entity_ref).action_status.twist.linear.x;
    };
  } else {
    throw UNSUPPORTED_SETTING_DETECTED(RelativeTargetSpeed, speed_target_value_type);
  }
}

auto RelativeTargetSpeed::getIsEnd() const -> std::function<bool(const EntityRef &)>
{
  if (continuous) {
    return [](const auto &) { return false; };  // ends never
  } else {
    return [calc_absolute_target_speed = getCalculateAbsoluteTargetSpeed()](const auto & actor) {
      try {
        const auto compare = Rule(Rule::equalTo);
        return compare(
          getEntityStatus(actor).action_status.twist.linear.x, calc_absolute_target_speed());
      } catch (const SemanticError &) {
        return false;  // NOTE: The actor is maybe lane-changing now
      }
    };
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
