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

#define OPENSCENARIO_INTERPRETER_NO_EXTENSION

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/catalog.hpp>
#include <openscenario_interpreter/syntax/controller.hpp>
#include <openscenario_interpreter/syntax/maneuver.hpp>
#include <openscenario_interpreter/syntax/misc_object.hpp>
#include <openscenario_interpreter/syntax/pedestrian.hpp>
#include <openscenario_interpreter/syntax/route.hpp>
#include <openscenario_interpreter/syntax/vehicle.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Catalog::Catalog(const pugi::xml_node & node, Scope & scope)
: name(readAttribute<std::string>("name", node, scope))
{
#define READ_CATEGORY(TYPE)                                  \
  traverse<0, 1>(node, #TYPE, [&](auto && each) {            \
    const auto element = make<TYPE>(each, scope);            \
    scope.insert(element.template as<TYPE>().name, element); \
  })

  READ_CATEGORY(Vehicle);
  READ_CATEGORY(Controller);
  READ_CATEGORY(Pedestrian);
  READ_CATEGORY(MiscObject);
  // READ_CATEGORY(Environment);
  READ_CATEGORY(Maneuver);
  // READ_CATEGORY(Trajectory);
  READ_CATEGORY(Route);

#undef READ_CATEGORY
}
}  // namespace syntax
}  // namespace openscenario_interpreter
