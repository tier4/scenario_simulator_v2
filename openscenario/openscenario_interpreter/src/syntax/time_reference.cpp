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
#include <openscenario_interpreter/syntax/none.hpp>
#include <openscenario_interpreter/syntax/time_reference.hpp>
#include <openscenario_interpreter/syntax/timing.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
TimeReference::TimeReference(const pugi::xml_node & node, Scope & scope)
// clang-format off
: ComplexType(
    choice(node, {
      {   "None", [&](const auto & node) { return make<  None>(node, scope); } },
      { "Timing", [&](const auto & node) { return make<Timing>(node, scope); } },
    }))
// clang-format on
{
}
}  // namespace syntax
}  // namespace openscenario_interpreter
