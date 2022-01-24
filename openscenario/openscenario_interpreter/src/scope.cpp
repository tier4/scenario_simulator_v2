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

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/range/algorithm.hpp>
#include <cassert>
#include <iterator>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <scenario_simulator_exception/exception.hpp>

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

auto EnvironmentFrame::lookdown(const std::string & name) const -> Object
{
  auto pass_through = [&](const auto & current_frames) {
    std::vector<const EnvironmentFrame *> result;

    for (auto && current_frame : current_frames) {
      std::copy(
        std::cbegin(current_frame->unnamed_inner_frames),
        std::cend(current_frame->unnamed_inner_frames), std::back_inserter(result));
    }

    return result;
  };

  for (std::vector<const EnvironmentFrame *> frames{this}; not frames.empty();) {
    std::vector<Object> result;

    for (auto && frame : frames) {
      boost::range::for_each(frame->variables.equal_range(name), [&](auto && name_and_value) {
        return result.push_back(name_and_value.second);
      });
    }

    switch (result.size()) {
      case 0:
        frames = pass_through(frames);
        break;
      case 1:
        return result.front();
      default:
        throw SyntaxError("Ambiguous reference to ", std::quoted(name));
    }
  }

  return Object();
}

auto EnvironmentFrame::isOutermost() const noexcept -> bool { return outer_frame == nullptr; }

auto EnvironmentFrame::resolveFrontPrefix(const Prefixed<Name> & prefixed_name) const
  -> std::list<const EnvironmentFrame *>
{
  std::list<const EnvironmentFrame *> result;

  auto range = inner_frames.equal_range(prefixed_name.prefixes.front());

  for (auto it = range.first; it != range.second; ++it) {
    result.push_back(it->second);
  }

  if (result.empty()) {
    for (auto & child : unnamed_inner_frames) {
      result.merge(child->resolveFrontPrefix(prefixed_name));
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
    auto sibling_scope = outer_frame->resolveFrontPrefix(prefixed_name);
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

Scope::Scope(const boost::filesystem::path & pathname)
: frame(new EnvironmentFrame()), global_environment(std::make_shared<GlobalEnvironment>(pathname))
{
}

Scope::Scope(const std::string & name, const Scope & outer)
: frame(std::shared_ptr<EnvironmentFrame>(new EnvironmentFrame(*outer.frame, name))),
  global_environment(outer.global_environment),
  name(name),
  actors(outer.actors)
{
}

auto Scope::global() const -> const GlobalEnvironment &
{
  assert(global_environment);
  return *global_environment;
}

auto Scope::global() -> GlobalEnvironment &
{
  assert(global_environment);
  return *global_environment;
}

auto Scope::local() const noexcept -> const Scope & { return *this; }

auto Scope::local() noexcept -> Scope & { return *this; }

auto Scope::insert(const Name & identifier, const Object & object) -> void
{
  return frame->define(identifier, object);
}

Scope::GlobalEnvironment::GlobalEnvironment(const boost::filesystem::path & pathname)
: pathname(pathname)
{
}

auto Scope::GlobalEnvironment::entityRef(const EntityRef & entity_ref) const -> Object
{
  try {
    return entities.at(entity_ref);
  } catch (const std::out_of_range &) {
    throw Error("An undeclared entity ", std::quoted(entity_ref), " was specified in entityRef.");
  }
}

auto Scope::GlobalEnvironment::isAddedEntity(const EntityRef & entity_ref) const -> bool
{
  return entityRef(entity_ref).as<ScenarioObject>().is_added;
}
}  // namespace openscenario_interpreter
