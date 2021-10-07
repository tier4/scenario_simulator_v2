// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#define OPENSCENARIO_INTERPRETER_ALLOW_ATTRIBUTES_TO_BE_BLANK
#define OPENSCENARIO_INTERPRETER_NO_EXTENSION

#include <openscenario_interpreter/syntax/catalog.hpp>

#include <openscenario_interpreter/syntax/controller.hpp>
#include <openscenario_interpreter/syntax/maneuver.hpp>
#include <openscenario_interpreter/syntax/misc_object.hpp>
#include <openscenario_interpreter/syntax/pedestrian.hpp>
#include <openscenario_interpreter/syntax/route.hpp>
#include <openscenario_interpreter/syntax/vehicle.hpp>
#include "openscenario_interpreter/reader/attribute.hpp"
#include "openscenario_interpreter/reader/element.hpp"
#include "scenario_simulator_exception/exception.hpp"

namespace openscenario_interpreter
{
inline namespace syntax
{
Catalog::Catalog(const pugi::xml_node & node, Scope & scope)
: name(readAttribute<std::string>("name", node, scope))
{
  if (scope.isTopLevel()) {
    bool already_found = false;

#define FIND_CATEGORY_ELEMENT(TYPE)                                                   \
  do {                                                                                \
    auto elements = readElementsAsElement<TYPE, 0>(#TYPE, node, scope);               \
    if (not elements.empty()) {                                                       \
      if (already_found) {                                                            \
        THROW_SYNTAX_ERROR("Only one type can be defined in a single category file"); \
      }                                                                               \
      already_found = true;                                                           \
      for (Element & element : elements) {                                            \
        scope.insert(element.template as<TYPE>().name, element);                      \
      }                                                                               \
    }                                                                                 \
  } while (0)

    FIND_CATEGORY_ELEMENT(Vehicle);
    FIND_CATEGORY_ELEMENT(Controller);
    FIND_CATEGORY_ELEMENT(Pedestrian);
    FIND_CATEGORY_ELEMENT(MiscObject);
    // FIND_CATEGORY_ELEMENT(Environment);
    FIND_CATEGORY_ELEMENT(Maneuver);
    // FIND_CATEGORY_ELEMENT(Trajectory);
    FIND_CATEGORY_ELEMENT(Route);
#undef FIND_CATEGORY_ELEMENT
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
