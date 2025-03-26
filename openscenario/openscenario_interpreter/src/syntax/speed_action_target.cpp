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

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/speed_action_target.hpp>
#include <openscenario_interpreter/utility/overload.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
SpeedActionTarget::SpeedActionTarget(const pugi::xml_node & node, Scope & scope)
// clang-format off
: ComplexType(
    choice(node,
      std::make_pair("RelativeTargetSpeed", [&](const auto & node) { return make<RelativeTargetSpeed>(node, scope); }),
      std::make_pair("AbsoluteTargetSpeed", [&](const auto & node) { return make<AbsoluteTargetSpeed>(node, scope); })))
// clang-format on
{
}
}  // namespace syntax
}  // namespace openscenario_interpreter
