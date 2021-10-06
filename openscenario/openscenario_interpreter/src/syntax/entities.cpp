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

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/entities.hpp>
#include <openscenario_interpreter/syntax/entity_selection.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Entities::Entities(const pugi::xml_node & node, Scope & scope)
{
  callWithElements(node, "ScenarioObject", 0, unbounded, [&](auto && node) {
    auto name = readAttribute<std::string>("name", node, scope);
    auto element = make<ScenarioObject>(node, scope);
    scope.global().entities.emplace(name, element);
    // scope.insert(name, element);
  });

  callWithElements(node, "EntitySelection", 0, unbounded, [&](auto && node) {
    throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name());
    return unspecified;
  });
}
}  // namespace syntax
}  // namespace openscenario_interpreter
