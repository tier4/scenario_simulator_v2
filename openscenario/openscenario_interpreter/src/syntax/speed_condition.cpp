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
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/entities.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <openscenario_interpreter/syntax/speed_condition.hpp>
#include <openscenario_interpreter/utility/print.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
SpeedCondition::SpeedCondition(
  const pugi::xml_node & node, Scope & scope, const TriggeringEntities & triggering_entities)
: Scope(scope),
  value(readAttribute<Double>("value", node, scope)),
  compare(readAttribute<Rule>("rule", node, scope)),
  triggering_entities(triggering_entities),
  results(triggering_entities.entity_refs.size(), Double::nan())
{
}

auto SpeedCondition::description() const -> String
{
  std::stringstream description;

  description << triggering_entities.description() << "'s speed = ";

  print_to(description, results);

  description << " " << compare << " " << value << "?";

  return description.str();
}

auto SpeedCondition::evaluate(const EntityRef & triggering_entity, const Entities * entities)
  -> double
{
  if (entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
    return evaluateSpeed(triggering_entity);
  } else {
    return Double::nan();
  }
}

auto SpeedCondition::evaluate() -> Object
{
  results.clear();

  return asBoolean(triggering_entities.apply([&](auto && triggering_entity) {
    results.push_back(evaluate(triggering_entity, global().entities));
    return compare(results.back(), value);
  }));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
