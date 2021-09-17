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

#include <openscenario_interpreter/syntax/speed_action_target.hpp>
#include <openscenario_interpreter/utility/overload.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto SpeedActionTarget::getCalculateAbsoluteTargetSpeed() const -> std::function<double()>
{
  return apply<std::function<double()>>(
    [](auto && target_speed) { return target_speed.getCalculateAbsoluteTargetSpeed(); }, *this);
}

auto SpeedActionTarget::getIsEnd() const -> std::function<bool(const EntityRef &)>
{
  using result = std::function<bool(const EntityRef &)>;

  return apply<result>(
    [](const auto & target_speed) -> result { return target_speed.getIsEnd(); }, *this);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
