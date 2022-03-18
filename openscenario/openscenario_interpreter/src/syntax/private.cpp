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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/private.hpp>
#include <openscenario_interpreter/utility/demangle.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Private::Private(const pugi::xml_node & node, Scope & scope)
: Scope(scope), entity_ref(readAttribute<String>("entityRef", node, local()))
{
  actors.emplace_back(entity_ref);

  traverse<1, unbounded>(node, "PrivateAction", [&](auto && node) {
    return private_actions.emplace_back(node, local());
  });
}

auto Private::endsImmediately() const -> bool
{
  return std::all_of(
    private_actions.begin(), private_actions.end(),
    [](const PrivateAction & private_action) { return private_action.endsImmediately(); });
}

auto Private::evaluate() -> Object
{
  assert(endsImmediately());
  for (auto && private_action : private_actions) {
    private_action.start();
    private_action.run();
  }
  return unspecified;
}

auto operator<<(nlohmann::json & json, const Private & datum) -> nlohmann::json &
{
  json["entityRef"] = datum.entity_ref;

  json["PrivateAction"] = nlohmann::json::array();

  for (const auto & private_action : datum.private_actions) {
    nlohmann::json action;
    action["type"] = makeTypename(private_action.type());
    json["PrivateAction"].push_back(action);
  }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
