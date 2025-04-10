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

#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/time_headway_condition.hpp>
#include <openscenario_interpreter/utility/print.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
TimeHeadwayCondition::TimeHeadwayCondition(
  const pugi::xml_node & node, Scope & scope, const TriggeringEntities & triggering_entities)
: entity_ref(readAttribute<String>("entityRef", node, scope), scope),
  value(readAttribute<Double>("value", node, scope)),
  freespace(readAttribute<Boolean>("freespace", node, scope)),
  along_route(readAttribute<Boolean>("alongRoute", node, scope)),
  compare(readAttribute<Rule>("rule", node, scope)),
  triggering_entities(triggering_entities),
  results(triggering_entities.entity_refs.size(), {Double::nan()})
{
}

auto TimeHeadwayCondition::description() const -> String
{
  std::stringstream description;

  description << triggering_entities.description()
              << "'s headway time between each and the referenced entity " << entity_ref << " = ";

  print_to(description, results);

  description << " " << compare << " " << value << "?";

  return description.str();
}

auto TimeHeadwayCondition::evaluate() -> Object
{
  results.clear();

  return asBoolean(triggering_entities.apply([&](auto && triggering_entity) {
    results.push_back(triggering_entity.apply(
      [&](const auto & object) { return evaluateTimeHeadway(entity_ref, object); }));
    return not results.back().size() or compare(results.back(), value).min();
  }));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
