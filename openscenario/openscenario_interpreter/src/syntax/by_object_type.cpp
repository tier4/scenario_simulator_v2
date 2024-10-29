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

#include <openscenario_interpreter/syntax/by_object_type.hpp>
#include <openscenario_interpreter/syntax/entities.hpp>
#include <openscenario_interpreter/syntax/entity.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <scenario_simulator_exception/exception.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
ByObjectType::ByObjectType(const pugi::xml_node & node, Scope & scope)
: Scope(scope), type(readAttribute<ObjectType>("type", node, scope))
{
  if (type.value == ObjectType::external) {
    THROW_SEMANTIC_ERROR("ObjectType::external does not support yet");
  }
}

auto ByObjectType::objects() const -> std::set<Entity>
{
  std::set<Entity> result;
  for (const auto & [name, object] : *global().entities) {
    if (object.is<ScenarioObject>() and object.as<ScenarioObject>().objectType() == type) {
      result.emplace(name, *this);
    }
  }
  return result;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
