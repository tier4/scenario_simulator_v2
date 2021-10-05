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

#include <openscenario_interpreter/syntax/controller.hpp>
#include <openscenario_interpreter/syntax/string.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Controller::Controller(const pugi::xml_node & node, Scope & scope)
: Scope(scope.makeChildScope(readAttribute<String>("name", node, scope))),
  parameter_declarations(
    readElement<ParameterDeclarations>("ParameterDeclarations", node, localScope())),
  properties(readElement<Properties>("Properties", node, localScope()))
{
}

Controller::operator openscenario_msgs::msg::DriverModel()
{
  openscenario_msgs::msg::DriverModel controller;
  {
    controller.see_around = not(*this)["isBlind"];
  }

  return controller;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
