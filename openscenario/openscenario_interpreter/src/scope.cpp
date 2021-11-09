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
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <scenario_simulator_exception/exception.hpp>

namespace openscenario_interpreter
{
EnvironmentFrame::EnvironmentFrame(EnvironmentFrame & parent, const std::string & name)
: qualifier(name), parent(&parent)
{
  if (name.empty()) {
    parent.anonymous_children.push_back(this);
  } else {
    parent.named_children.emplace(name, this);
  }
}

auto EnvironmentFrame::findObject(const QualifiedIdentifier & identifier) const -> Object
{
  auto split = identifier.qualifiers;

  split.push_back(identifier.name);

  if (split.size() == 1) {
    return lookupUnqualifiedElement(split.front());
  } else {
    auto top_scope = lookupUnqualifiedScope(split.front());
    if (top_scope) {
      return lookupQualifiedElement(top_scope, split.begin() + 1, split.end());
    } else {
      return unspecified;
    }
  }
}

auto EnvironmentFrame::fullyQualifiedName() const -> std::string
{
  std::string result;

  for (const auto * frame = this; frame; frame = frame->parent) {
    result = (frame->qualifier.empty() ? std::string("{annonymous}") : frame->qualifier) + result;
  }

  return "::" + result;
}

auto EnvironmentFrame::define(const UnqualifiedIdentifier & identifier, const Object & element)
  -> void
{
  environments.emplace(identifier.name, element);
}

auto EnvironmentFrame::lookupChildElement(const std::string & name) const -> Object
{
  std::vector<const EnvironmentFrame *> same_level{this};

  while (not same_level.empty()) {
    std::vector<const EnvironmentFrame *> next_level;
    std::vector<Object> ret;

    for (auto * frame : same_level) {
      auto range = frame->environments.equal_range(name);
      for (auto it = range.first; it != range.second; ++it) {
        ret.push_back(it->second);
      }

      for (auto * f : frame->anonymous_children) {
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
  return Object{};
}

auto EnvironmentFrame::lookupChildScope(const std::string & name) const
  -> std::list<const EnvironmentFrame *>
{
  auto range = named_children.equal_range(name);
  std::list<const EnvironmentFrame *> ret;
  if (range.first != range.second) {
    for (auto it = range.first; it != range.second; ++it) {
      ret.push_back(it->second);
    }
  } else {
    for (auto & child : anonymous_children) {
      ret.merge(child->lookupChildScope(name));
    }
  }
  return ret;
}

auto EnvironmentFrame::lookupQualifiedElement(
  const EnvironmentFrame * scope,                       //
  std::vector<std::string>::const_iterator name_begin,  //
  std::vector<std::string>::const_iterator name_end) -> Object
{
  for (auto iter = name_begin; iter != name_end - 1; ++iter) {
    auto found = scope->lookupChildScope(*iter);
    if (found.size() == 1) {
      scope = found.front();
    } else if (found.empty()) {
      return Object{};
    } else if (found.size() > 1) {
      THROW_SYNTAX_ERROR("ambiguous reference to ", std::quoted(*iter));
    }
  }

  return scope->lookupChildElement(*(name_end - 1));
}

auto EnvironmentFrame::lookupUnqualifiedElement(const std::string & name) const -> Object
{
  for (auto * p = this; p != nullptr; p = p->parent) {
    auto found = p->lookupChildElement(name);
    if (found) {
      return found;
    }
  }
  return Object{};
}

auto EnvironmentFrame::lookupUnqualifiedScope(const std::string & name) const
  -> const EnvironmentFrame *
{
  if (parent == nullptr) {  // this is global scope
    return name.empty() ? this : nullptr;
  } else {
    auto sibling_scope = parent->lookupChildScope(name);
    if (sibling_scope.size() == 1) {
      return sibling_scope.front();
    } else if (sibling_scope.size() > 1) {
      THROW_SYNTAX_ERROR("ambiguous reference to ", name);
    } else if (sibling_scope.empty() && parent) {
      return parent->lookupUnqualifiedScope(name);
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

auto Scope::insert(const UnqualifiedIdentifier & identifier, const Object & object) -> void
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
