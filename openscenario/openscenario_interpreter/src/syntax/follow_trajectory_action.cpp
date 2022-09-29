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

#include <openscenario_interpreter/syntax/follow_trajectory_action.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
FollowTrajectoryAction::FollowTrajectoryAction(const pugi::xml_node & node, Scope & scope)
: initial_distance_offset(
    readAttribute<Double>("initialDistanceOffset", node, scope))
{
}

auto FollowTrajectoryAction::accomplished() noexcept -> bool { return false; }

auto FollowTrajectoryAction::endsImmediately() noexcept -> bool { return false; }

auto FollowTrajectoryAction::run() -> void {}

auto FollowTrajectoryAction::start() -> void {}
}  // namespace syntax
}  // namespace openscenario_interpreter
