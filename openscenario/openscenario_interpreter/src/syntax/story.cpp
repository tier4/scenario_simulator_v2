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
#include <openscenario_interpreter/syntax/act.hpp>
#include <openscenario_interpreter/syntax/parameter_declarations.hpp>
#include <openscenario_interpreter/syntax/story.hpp>
#include <openscenario_interpreter/syntax/string.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Story::Story(const pugi::xml_node & node, Scope & scope)
: Scope(readAttribute<String>("name", node, scope), scope)
{
  callWithElements(node, "ParameterDeclarations", 0, 1, [&](auto && node) {
    return make<ParameterDeclarations>(node, local());
  });

  callWithElements(node, "Act", 1, unbounded, [&](auto && node) {
    return acts.push_back(readStoryboardElement<Act>(node, local()));
  });
}

auto Story::accomplished() const -> bool
{
  // NOTE: A Story's goal is accomplished when all its Acts are in the completeState.
  return std::all_of(std::begin(acts), std::end(acts), [](auto && act) {
    return act.template as<Act>().complete();
  });
}

auto Story::elements() -> Elements & { return acts; }

auto Story::ready() noexcept -> bool { return true; }

auto Story::run() -> void
{
  for (auto && act : acts) {
    act.evaluate();
  }
}

auto Story::start() noexcept -> void {}

auto Story::stop() -> void
{
  for (auto && act : acts) {
    act.as<Act>().override();
    act.evaluate();
  }
}

auto Story::stopTriggered() noexcept -> bool { return false; }

auto operator<<(nlohmann::json & json, const Story & story) -> nlohmann::json &
{
  json["name"] = story.name;

  json["currentState"] = boost::lexical_cast<std::string>(story.state());

  json["Act"] = nlohmann::json::array();

  for (auto && act : story.acts) {
    nlohmann::json json_act;
    json_act << act.as<Act>();
    json["Act"].push_back(json_act);
  }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
