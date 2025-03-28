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

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <cassert>
#include <iterator>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/open_scenario.hpp>
#include <scenario_simulator_exception/exception.hpp>

#ifndef PARAMETER_VALUE_DISTRIBUTION_ONLY
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#endif  // PARAMETER_VALUE_DISTRIBUTION_ONLY

namespace openscenario_interpreter
{
EnvironmentFrame::EnvironmentFrame(EnvironmentFrame & outer_frame, const std::string & name)
: outer_frame(&outer_frame)
{
  if (name.empty()) {
    outer_frame.unnamed_inner_frames.push_back(this);
  } else {
    outer_frame.inner_frames.emplace(name, this);
  }
}

auto EnvironmentFrame::define(const Name & name, const Object & object) -> void
{
  variables.emplace(name, object);
}

auto EnvironmentFrame::isOutermost() const noexcept -> bool { return outer_frame == nullptr; }

auto EnvironmentFrame::resolvePrefix(const Prefixed<Name> & prefixed_name) const
  -> std::list<const EnvironmentFrame *>
{
  std::list<const EnvironmentFrame *> result;

  boost::range::for_each(
    inner_frames.equal_range(prefixed_name.prefixes.front()),
    [&](auto && name_and_frame) { result.push_back(name_and_frame.second); });

  if (result.empty()) {
    // BUG: must be breadth first search
    for (auto & child : unnamed_inner_frames) {
      result.merge(child->resolvePrefix(prefixed_name));
    }
  }

  return result;
}

auto EnvironmentFrame::outermostFrame() const noexcept -> const EnvironmentFrame &
{
  return isOutermost() ? *this : outer_frame->outermostFrame();
}

auto EnvironmentFrame::lookupFrame(const Prefixed<Name> & prefixed_name) const
  -> const EnvironmentFrame *
{
  assert(not prefixed_name.prefixes.empty());

  if (isOutermost()) {
    return this;
  } else {
    auto sibling_scope = outer_frame->resolvePrefix(prefixed_name);
    switch (sibling_scope.size()) {
      case 0:
        assert(outer_frame);
        return outer_frame->lookupFrame(prefixed_name);
      case 1:
        assert(sibling_scope.front());
        return sibling_scope.front();
      default:
        throw SyntaxError(
          "There are multiple StoryboardElements that can be referenced by the name ",
          std::quoted(boost::lexical_cast<std::string>(prefixed_name)), ".");
    }
  }
}

Scope::Scope(const OpenScenario * const open_scenario)
: open_scenario(open_scenario),
  frame(new EnvironmentFrame()),
  scenario_definition(std::make_shared<ScenarioDefinition>())
{
}

Scope::Scope(const std::string & name, const Scope & outer)
: open_scenario(outer.open_scenario),
  frame(std::shared_ptr<EnvironmentFrame>(new EnvironmentFrame(*outer.frame, name))),
  scenario_definition(outer.scenario_definition),
  name(name)
#ifndef PARAMETER_VALUE_DISTRIBUTION_ONLY
  ,
  actors(outer.actors)
#endif  // PARAMETER_VALUE_DISTRIBUTION_ONLY
{
}

auto Scope::dirname() const -> std::string
{
  assert(open_scenario);
  return open_scenario->pathname.parent_path().string();
}

auto Scope::global() const -> const ScenarioDefinition &
{
  assert(scenario_definition);
  return *scenario_definition;
}

auto Scope::global() -> ScenarioDefinition &
{
  assert(scenario_definition);
  return *scenario_definition;
}

auto Scope::local() const noexcept -> const Scope & { return *this; }

auto Scope::local() noexcept -> Scope & { return *this; }

auto Scope::insert(const Name & identifier, const Object & object) -> void
{
  return frame->define(identifier, object);
}
}  // namespace openscenario_interpreter
