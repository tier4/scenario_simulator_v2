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
#include <openscenario_interpreter/syntax/acceleration_condition.hpp>
#include <openscenario_interpreter/utility/print.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
AccelerationCondition::AccelerationCondition(
  const pugi::xml_node & node, Scope & scope, const TriggeringEntities & triggering_entities)
: value(readAttribute<Double>("value", node, scope)),
  compare(readAttribute<Rule>("rule", node, scope)),
  triggering_entities(triggering_entities),
  results(triggering_entities.entity_refs.size(), Double::nan())
{
}

auto AccelerationCondition::description() const -> std::string
{
  std::stringstream description;

  description << triggering_entities.description() << "'s acceleration = ";

  print_to(description, results);

  description << " " << compare << " " << value << "?";

  return description.str();
}

auto AccelerationCondition::evaluate() -> Object
{
  results.clear();

  return asBoolean(triggering_entities.apply([&](auto && triggering_entity) {
    results.push_back(getEntityStatus(triggering_entity).action_status.accel.linear.x);
    return compare(results.back(), value);
  }));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
