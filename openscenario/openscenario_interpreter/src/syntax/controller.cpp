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
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/controller.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/string.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Controller::Controller(const pugi::xml_node & node, Scope & scope)
: Scope(readAttribute<String>("name", node, scope), scope),
  parameter_declarations(
    readElement<ParameterDeclarations>("ParameterDeclarations", node, local())),
  properties(readElement<Properties>("Properties", node, local()))
{
}

auto Controller::assign(const EntityRef & entity_ref) -> void
{
  const auto max_speed = properties["maxSpeed"];

  if (not max_speed.value.empty()) {
    setUpperBoundSpeed(entity_ref, Double(max_speed.value));
  }

  applyAssignControllerAction(entity_ref, [&]() {
    auto message = connection.getDriverModel(entity_ref);
    if (properties.find("isBlind") != std::end(properties)) {
      message.see_around = not properties["isBlind"];
    }
    return message;
  }());
}

auto Controller::isUserDefinedController() & -> bool
{
  return static_cast<bool>(properties["isEgo"]);
}

auto Controller::operator[](const String & name) -> const Property &  //
{
  return properties[name];
}
}  // namespace syntax
}  // namespace openscenario_interpreter
