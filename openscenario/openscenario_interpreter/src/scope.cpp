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

auto EnvironmentFrame::find(const Name & name) const -> Object
{
  for (auto frame = this; frame; frame = frame->outer_frame) {
    auto object = frame->lookdown(name);
    if (object) {
      return object;
    }
  }

  return Object();
}

auto EnvironmentFrame::findObject(const PrefixedName & prefixed_name) const -> Object
{
  auto split = prefixed_name.prefixes;

  split.push_back(prefixed_name.name);

  if (prefixed_name.prefixes.empty()) {
    return find(prefixed_name.name);
  } else {
    auto top_scope = lookupUnqualifiedScope(split.front());
    if (top_scope) {
      return lookupQualifiedElement(top_scope, split.begin() + 1, split.end());
    } else {
      return unspecified;
    }
  }
}

auto EnvironmentFrame::lookdown(const std::string & name) const -> Object
{
  std::vector<const EnvironmentFrame *> same_level{this};

  while (not same_level.empty()) {
    std::vector<const EnvironmentFrame *> next_level;

    std::vector<Object> ret;

    for (auto * frame : same_level) {
      auto range = frame->variables.equal_range(name);
      for (auto it = range.first; it != range.second; ++it) {
        ret.push_back(it->second);
      }

      for (auto * f : frame->unnamed_inner_frames) {
        next_level.push_back(f);
      }
    }

    if (ret.size() == 1) {
      return ret.front();
    }

    if (ret.size() > 1) {
      THROW_SYNTAX_ERROR("ambiguous reference to ", std::quoted(name));
    }

    same_level = std::move(next_level);
  }

  return Object();
}

auto EnvironmentFrame::lookupQualifiedElement(
  const EnvironmentFrame * scope,                  //
  std::vector<std::string>::const_iterator begin,  //
  std::vector<std::string>::const_iterator end) -> Object
{
  for (auto iter = begin; iter != end - 1; ++iter) {
    auto found = scope->frames(*iter);
    switch (found.size()) {
      case 0:
        return Object();
      case 1:
        scope = found.front();
        break;
      default:
        throw SyntaxError("Ambiguous reference to ", std::quoted(*iter), ".");
    }
  }

  return scope->lookdown(*(end - 1));
}

auto EnvironmentFrame::isOutermost() const noexcept -> bool { return outer_frame == nullptr; }

auto EnvironmentFrame::frames(const std::string & name) const
  -> std::list<const EnvironmentFrame *>
{
  std::list<const EnvironmentFrame *> result;

  auto range = inner_frames.equal_range(name);

  for (auto it = range.first; it != range.second; ++it) {
    result.push_back(it->second);
  }

  if (result.empty()) {
    for (auto & child : unnamed_inner_frames) {
      result.merge(child->frames(name));
    }
  }

  return result;
}

auto EnvironmentFrame::lookupUnqualifiedScope(const Name & name) const -> const EnvironmentFrame *
{
  if (isOutermost()) {
    return name.empty() ? this : nullptr;
  } else {
    auto sibling_scope = outer_frame->frames(name);
    if (sibling_scope.size() == 1) {
      return sibling_scope.front();
    } else if (sibling_scope.size() > 1) {
      throw SyntaxError("ambiguous reference to ", std::quoted(name));
    } else if (sibling_scope.empty() && outer_frame) {
      return outer_frame->lookupUnqualifiedScope(name);
    }
    return nullptr;
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

auto Scope::findObject(const std::string & name_) const -> Object
{
  return frame->findObject(name_);
}

auto Scope::global() const -> const GlobalEnvironment & { return *global_environment; }

auto Scope::global() -> GlobalEnvironment & { return *global_environment; }

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
