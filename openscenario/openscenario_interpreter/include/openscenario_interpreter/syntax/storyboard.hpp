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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_HPP_

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/init.hpp>
#include <openscenario_interpreter/syntax/story.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Storyboard -------------------------------------------------------------
 *
 *  <xsd:complexType name="Storyboard">
 *    <xsd:sequence>
 *      <xsd:element name="Init" type="Init"/>
 *      <xsd:element name="Story" maxOccurs="unbounded" type="Story"/>
 *      <xsd:element name="StopTrigger" type="Trigger"/>
 *    </xsd:sequence>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Storyboard : public StoryboardElement<Storyboard>, public Elements
{
  Scope inner_scope;

  Init init;

  Trigger stop_trigger;

  const String name{"Storyboard"};

  template <typename Node, typename Scope>
  explicit Storyboard(const Node & node, Scope & outer_scope)
  : inner_scope(outer_scope),
    init(readElement<Init>("Init", node, inner_scope)),
    stop_trigger(readElement<Trigger>("StopTrigger", node, inner_scope))
  {
    callWithElements(node, "Story", 1, unbounded, [&](auto && node) {
      return push_back(readStoryboardElement<Story>(node, inner_scope));
    });
  }

  const std::true_type ready{};

  void start()
  {
    for (const auto & each : inner_scope.entities) {
      std::get<1>(each).evaluate();
    }

    init.evaluate();  // NOTE RENAME TO 'start'?
  }

  decltype(auto) stopTriggered() { return stop_trigger.evaluate().as<Boolean>(); }

  void stop()
  {
    for (auto && each : *this) {
      each.as<Story>().override();
      each.evaluate();
    }
  }

  auto accomplished() const
  {
    auto check = [](auto && each) { return each.template as<Story>().complete(); };

    return std::all_of(std::begin(*this), std::end(*this), check);
  }

  bool engaged = false;

  auto run()
  {
    const auto all_ready = std::all_of(
      std::begin(inner_scope.entities), std::end(inner_scope.entities), [&](const auto & each) {
        // DEBUG_VALUE(each.first);
        // DEBUG_VALUE(each.second.template as<ScenarioObject>().template is<Vehicle>());
        // DEBUG_VALUE(each.second.template as<ScenarioObject>().object_controller.isEgo());
        // DEBUG_VALUE(not openscenario_interpreter::ready(each.first));
        // return each.second.template is<Vehicle>() and                                   //
        //        each.second.template as<ScenarioObject>().object_controller.isEgo() and  //
        //        not openscenario_interpreter::ready(each.first);
        return openscenario_interpreter::ready(each.first);
      });

    // DEBUG_VALUE(getCurrentTime());
    // DEBUG_VALUE(all_ready);

    if (0 <= getCurrentTime()) {
      if (engaged) {
        for (auto && story : *this) {
          story.evaluate();
        }
      } else if (all_ready) {
        for (const auto & each : inner_scope.entities) {
          engage(each.first);
        }
        engaged = true;
      } else {
        throw common::AutowareError(
          "Autoware does not respond. It is likely that some nodes were corrupted during launch");
      }
    }
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_HPP_
