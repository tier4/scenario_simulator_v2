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

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <openscenario_interpreter/syntax/story.hpp>
#include <openscenario_interpreter/syntax/storyboard.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Storyboard::Storyboard(const pugi::xml_node & node, Scope & scope)
: Scope("Storyboard", scope),  // FIXME DIRTY HACK
  init(readElement<Init>("Init", node, local())),
  stop_trigger(readElement<Trigger>("StopTrigger", node, local()))
{
  callWithElements(node, "Story", 1, unbounded, [&](auto && node) {
    return push_back(readStoryboardElement<Story>(node, local()));
  });

  if (not init.endsImmediately()) {
    throw SemanticError("Init.Actions should end immediately");
  }
}

auto Storyboard::accomplished() const -> bool
{
  return std::all_of(std::begin(*this), std::end(*this), [](auto && each) {
    return each.template as<Story>().complete();
  });
}

auto Storyboard::ready() noexcept -> bool { return true; }

auto Storyboard::run() -> void
{
  if (engaged) {
    for (auto && story : *this) {
      story.evaluate();
    }
  } else if (std::all_of(  // XXX DIRTY HACK!!!
               std::cbegin(global().entities), std::cend(global().entities),
               [&](const auto & each) {
                 return not std::get<1>(each).template as<ScenarioObject>().is_added or
                        openscenario_interpreter::ready(std::get<0>(each));
               })) {
    for (const auto & each : global().entities) {
      if (std::get<1>(each).template as<ScenarioObject>().is_added) {
        engage(std::get<0>(each));
      }
    }
    engaged = true;
  } else {
    throw common::AutowareError(
      "Autoware did not reach an engageable state within the specified time "
      "(initialize_duration).");
  }
}

auto Storyboard::start() -> void
{
  init.evaluate();  // NOTE RENAME TO 'start'?
}

auto Storyboard::stop() -> void
{
  for (auto && each : *this) {
    each.as<Story>().override();
    each.evaluate();
  }
}

auto Storyboard::stopTriggered() -> bool { return stop_trigger.evaluate().as<Boolean>(); }

auto operator<<(nlohmann::json & json, const Storyboard & datum) -> nlohmann::json &
{
  json["currentState"] = boost::lexical_cast<std::string>(datum.currentState());

  json["Init"] << datum.init;

  json["Story"] = nlohmann::json::array();

  for (const auto & each : datum) {
    nlohmann::json story;
    story << each.as<Story>();
    json["Story"].push_back(story);
  }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
