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
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/catalog_locations.hpp>
#include <tuple>

namespace openscenario_interpreter
{
inline namespace syntax
{
#define ELEMENT(TYPE)                                                                              \
  traverse<0, 1>(node, #TYPE "Catalog", [&](auto && node) {                                        \
    return emplace(                                                                                \
      std::piecewise_construct, std::forward_as_tuple(#TYPE), std::forward_as_tuple(node, scope)); \
  })

CatalogLocations::CatalogLocations(const pugi::xml_node & node, Scope & scope)
{
  ELEMENT(Vehicle);
  ELEMENT(Controller);
  ELEMENT(Pedestrian);
  ELEMENT(MiscObject);
  ELEMENT(Environment);
  ELEMENT(Maneuver);
  ELEMENT(Trajectory);
  ELEMENT(Route);

  scope.global().catalog_locations = this;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
