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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/triggering_entities.hpp>
#include <openscenario_interpreter/utility/print.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
TriggeringEntities::TriggeringEntities(const pugi::xml_node & node, Scope & scope)
: triggering_entities_rule(
    readAttribute<TriggeringEntitiesRule>("triggeringEntitiesRule", node, scope)),
  entity_refs(readElements<Entity, 1>("EntityRef", node, scope))
{
}

auto TriggeringEntities::description() const -> String
{
  std::stringstream description;

  description << triggering_entities_rule.description() << " ";

  print_to(description, entity_refs);

  return description.str();
}
}  // namespace syntax
}  // namespace openscenario_interpreter
