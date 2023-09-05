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
#include <openscenario_interpreter/syntax/act.hpp>
#include <openscenario_interpreter/syntax/custom_command_action.hpp>
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
  traverse<0, 1>(node, "ParameterDeclarations", [&](auto && node) {
    return make<ParameterDeclarations>(node, local());
  });

  traverse<1, unbounded>(node, "Act", [&](auto && node) {
    return elements.push_back(readStoryboardElement<Act>(node, local()));
  });
}

auto Story::run() -> void
{
  std::size_t index{0};
  for (auto && act : elements) {
    try {
      assert(act.is_also<Act>());
      act.evaluate();
      ++index;
    } catch (const SpecialAction<EXIT_FAILURE> & action) {
      throw SpecialAction<EXIT_FAILURE>(name, "Act", index, action);
    }
  }
}

auto operator<<(nlohmann::json & json, const Story & story) -> nlohmann::json &
{
  json["name"] = story.name;

  json["currentState"] = boost::lexical_cast<std::string>(story.state());

  json["Act"] = nlohmann::json::array();

  for (auto && act : story.elements) {
    nlohmann::json json_act;
    json_act << act.as<Act>();
    json["Act"].push_back(json_act);
  }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
