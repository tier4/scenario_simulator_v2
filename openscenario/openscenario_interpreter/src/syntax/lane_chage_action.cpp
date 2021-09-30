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
#include <openscenario_interpreter/syntax/lane_change_action.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto LaneChangeAction::accomplished() -> bool
{
  if (lane_change_target.is<AbsoluteTargetLane>()) {
    for (auto && each : accomplishments) {
      if (not std::get<1>(each)) {
        std::get<1>(each) = isInLanelet(
          std::get<0>(each), Integer(lane_change_target.as<AbsoluteTargetLane>().value), 0.1);
      }
    }
    return std::all_of(std::begin(accomplishments), std::end(accomplishments), cdr);
  } else {
    // NOTE: Specifying an unsupported element is an error in the constructor, so this line cannot be reached.
    throw UNSUPPORTED_ELEMENT_SPECIFIED(lane_change_target.type().name());
  }
}

auto LaneChangeAction::endsImmediately() noexcept -> bool { return false; }

auto LaneChangeAction::run() noexcept -> void {}

auto LaneChangeAction::start() -> void
{
  accomplishments.clear();

  if (lane_change_target.is<AbsoluteTargetLane>()) {
    for (const auto & actor : actors) {
      accomplishments.emplace(actor, false);
      applyLaneChangeAction(actor, Integer(lane_change_target.as<AbsoluteTargetLane>().value));
    }
  } else {
    // NOTE: Specifying an unsupported element is an error in the constructor, so this line cannot be reached.
    throw UNSUPPORTED_ELEMENT_SPECIFIED(lane_change_target.type().name());
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
