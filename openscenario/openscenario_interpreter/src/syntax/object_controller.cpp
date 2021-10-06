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
#include <openscenario_interpreter/syntax/controller.hpp>
#include <openscenario_interpreter/syntax/object_controller.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
int ObjectController::ego_count = 0;

ObjectController::ObjectController(const pugi::xml_node & node, Scope & scope)
// clang-format off
: ComplexType(
    choice(node,
      std::make_pair("CatalogReference", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
      std::make_pair("Controller",       [&](auto && node) { return make<Controller>(node, scope); })))
// clang-format on
{
  if (isEgo()) {
    ego_count++;
  }
}

ObjectController::ObjectController() : ComplexType(unspecified) {}

ObjectController::~ObjectController()
{
  if (isEgo()) {
    ego_count--;
  }
}

auto ObjectController::isEgo() const & -> bool
{
  if (is<Unspecified>()) {
    static auto controller = DefaultController();
    return static_cast<bool>(controller["isEgo"]);
  } else {
    return static_cast<bool>(as<Controller>()["isEgo"]);
  }
}

ObjectController::operator openscenario_msgs::msg::DriverModel() const
{
  if (is<Unspecified>()) {
    openscenario_msgs::msg::DriverModel controller;
    {
      controller.see_around = not DefaultController()["isBlind"];
    }
    return controller;
  } else {
    return openscenario_msgs::msg::DriverModel(as<Controller>());
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
