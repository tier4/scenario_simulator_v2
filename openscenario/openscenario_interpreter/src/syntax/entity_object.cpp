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

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/catalog_reference.hpp>
#include <openscenario_interpreter/syntax/entity_object.hpp>
#include <openscenario_interpreter/syntax/object_type.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
EntityObject::EntityObject(const pugi::xml_node & node, Scope & scope)
// clang-format off
: Group(
    choice(node,
      std::make_pair("CatalogReference", [&](auto && node) { return CatalogReference(node, scope).make(); }),
      std::make_pair("Vehicle",          [&](auto && node) { return make<Vehicle   >(node, scope);        }),
      std::make_pair("Pedestrian",       [&](auto && node) { return make<Pedestrian>(node, scope);        }),
      std::make_pair("MiscObject",       [&](auto && node) { return make<MiscObject>(node, scope);        })))
// clang-format on
{
}

auto EntityObject::objectType() const -> ObjectType::value_type
{
  return apply<ObjectType::value_type>(
    [](const auto & object) { return object.object_type; }, *this);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
