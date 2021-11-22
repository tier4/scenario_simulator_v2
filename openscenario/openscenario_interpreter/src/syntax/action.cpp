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
#include <openscenario_interpreter/syntax/action.hpp>
#include <openscenario_interpreter/utility/demangle.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Action::Action(const pugi::xml_node & node, Scope & scope)
// clang-format off
: Scope(readAttribute<String>("name", node, scope), scope),
  ComplexType(
    choice(node,
      std::make_pair(     "GlobalAction", [this](auto && node) { return make<     GlobalAction>(node, local()); }),
      std::make_pair("UserDefinedAction", [this](auto && node) { return make<UserDefinedAction>(node, local()); }),
      std::make_pair(    "PrivateAction", [this](auto && node) { return make<    PrivateAction>(node, local()); })))
// clang-format on
{
}

auto Action::ready() const -> bool { return static_cast<bool>(*this); }

auto Action::run() -> void
{
  return apply<void>([](auto && action) { return action.run(); }, *this);
}

auto Action::start() -> void
{
  return apply<void>([](auto && action) { return action.start(); }, *this);
}

auto Action::stop() -> void
{
  if (overridden) {
    current_state = complete_state;
  } else {
    overridden = true;
  }
}

auto Action::stopTriggered() noexcept -> bool { return false; }

auto operator<<(nlohmann::json & json, const Action & datum) -> nlohmann::json &
{
  json["name"] = datum.name;

  json["currentState"] = boost::lexical_cast<std::string>(datum.currentState());

  json["type"] =
    apply<std::string>([](auto && action) { return makeTypename(action.type()); }, datum);

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
