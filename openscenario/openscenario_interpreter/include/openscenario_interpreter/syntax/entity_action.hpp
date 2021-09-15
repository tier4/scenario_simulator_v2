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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_ACTION_HPP_

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/add_entity_action.hpp>
#include <openscenario_interpreter/syntax/delete_entity_action.hpp>
#include <typeindex>
#include <unordered_map>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- EntityAction -----------------------------------------------------------
 *
 *  <xsd:complexType name="EntityAction">
 *    <xsd:choice>
 *      <xsd:element name="AddEntityAction" type="AddEntityAction"/>
 *      <xsd:element name="DeleteEntityAction" type="DeleteEntityAction"/>
 *    </xsd:choice>
 *    <xsd:attribute name="entityRef" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct EntityAction : public Element
{
  const String entity_ref;

  template <typename Node, typename Scope>
  explicit EntityAction(const Node & node, Scope & outer_scope)
  // clang-format off
  : Element(
      choice(node,
        std::make_pair(   "AddEntityAction", [&](auto && node) { return make<   AddEntityAction>(node, outer_scope); }),
        std::make_pair("DeleteEntityAction", [&](auto && node) { return make<DeleteEntityAction>(node, outer_scope); }))),
    entity_ref(readAttribute<String>("entityRef", node, outer_scope))
  // clang-format on
  {
  }

  static auto accomplished() noexcept -> bool { return endsImmediately(); }

  static auto endsImmediately() noexcept -> bool { return true; }

  inline auto run() const -> void
  {
    // clang-format off
    static const std::unordered_map<std::type_index, std::function<void(const EntityAction &, const String &)>> overloads
    {
      { typeid(   AddEntityAction), [](const EntityAction & entity_action, auto &&... xs) { return entity_action.as<   AddEntityAction>()(std::forward<decltype(xs)>(xs)...); }},
      { typeid(DeleteEntityAction), [](const EntityAction & entity_action, auto &&... xs) { return entity_action.as<DeleteEntityAction>()(std::forward<decltype(xs)>(xs)...); }},
    };
    // clang-format on

    overloads.at(type())(*this, entity_ref);  // TODO(yamacir-kit) CATCH std::out_of_range
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_ACTION_HPP_
