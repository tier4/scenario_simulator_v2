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
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/custom_command_action.hpp>
#include <openscenario_interpreter/syntax/entities.hpp>  // TEMPORARY (TODO REMOVE THIS LINE)
#include <openscenario_interpreter/syntax/global_action.hpp>
#include <openscenario_interpreter/syntax/private.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <openscenario_interpreter/syntax/story.hpp>
#include <openscenario_interpreter/syntax/storyboard.hpp>
#include <openscenario_interpreter/syntax/user_defined_action.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Storyboard::Storyboard(const pugi::xml_node & node, Scope & scope)
: Scope("Storyboard", scope),  // FIXME DIRTY HACK
  StoryboardElement(readElement<Trigger>("StopTrigger", node, local())),
  init(readElement<Init>("Init", node, local()))
{
  elements.push_back(make(init.actions));
  traverse<1, unbounded>(node, "Story", [&](auto && node) {
    return elements.push_back(readStoryboardElement<Story>(node, local()));
  });

  while (not thunks.empty()) {
    std::invoke(thunks.front());
    thunks.pop();
  }
}

auto Storyboard::run() -> void
{
  std::size_t index{0};
  for (auto && story : elements) {
    try {
      story.evaluate();
      if (story.is<Story>()) {
        ++index;
      }
    } catch (const SpecialAction<EXIT_FAILURE> & action) {
      throw SpecialAction<EXIT_FAILURE>(name, "Story", index, action);
    }
  }
}

auto operator<<(nlohmann::json & json, const Storyboard & datum) -> nlohmann::json &
{
  json["currentState"] = boost::lexical_cast<std::string>(datum.state());

  json["Init"] << datum.init;

  json["Story"] = nlohmann::json::array();

  for (const auto & story : datum.elements) {
    nlohmann::json each;
    if (story.is<InitActions>()) {
      continue;
    }
    each << story.as<Story>();
    json["Story"].push_back(each);
  }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
