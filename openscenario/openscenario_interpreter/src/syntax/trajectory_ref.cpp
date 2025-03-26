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

#include <openscenario_interpreter/syntax/catalog_reference.hpp>
#include <openscenario_interpreter/syntax/trajectory.hpp>
#include <openscenario_interpreter/syntax/trajectory_ref.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
TrajectoryRef::TrajectoryRef(const pugi::xml_node & node, Scope & scope)
// clang-format off
: trajectory(
    choice(node,
      std::make_pair(      "Trajectory", [&](const auto & node) { return make<Trajectory>(node, scope);        }),
      std::make_pair("CatalogReference", [&](const auto & node) { return CatalogReference(node, scope).make(); })))
// clang-format on
{
}
}  // namespace syntax
}  // namespace openscenario_interpreter
