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

#ifndef OPENSCENARIO_INTERPRETER__SCOPE_HPP_
#define OPENSCENARIO_INTERPRETER__SCOPE_HPP_

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <limits>
#include <memory>
#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

namespace openscenario_interpreter
{
struct EnvironmentFrame
{
private:
  friend struct Scope;

  const std::string scope_name;

  std::unordered_multimap<std::string, Element> environments;

  EnvironmentFrame * const parent = nullptr;

  std::unordered_multimap<std::string, EnvironmentFrame *> named_children;

  std::vector<EnvironmentFrame *> anonymous_children;

  explicit EnvironmentFrame() = default;

  explicit EnvironmentFrame(EnvironmentFrame & parent, const std::string & name)
  : scope_name(name), parent(&parent)
  {
    if (name.empty()) {
      parent.anonymous_children.push_back(this);
    } else {
      parent.named_children.emplace(name, this);
    }
  }

  explicit EnvironmentFrame(const EnvironmentFrame &) = delete;

  explicit EnvironmentFrame(EnvironmentFrame &&) = delete;

public:
  auto insert(const std::string & name, Element element) -> void
  {
    if (name.find(':') != std::string::npos) {
      THROW_SYNTAX_ERROR("Identifier '", name, "' contains ':'");
    }

    environments.emplace(name, std::move(element));
  }

  auto findElement(const std::string & name) const -> Element
  {
    std::vector<std::string> split;
    {
      const char * delim = "::";
      const std::size_t delim_len = 2;

      std::size_t prev_pos = 0;
      std::size_t pos = 0;
      while ((pos = name.find(delim, prev_pos)) != std::string::npos) {
        split.push_back(name.substr(prev_pos, pos - prev_pos));
        prev_pos = pos + delim_len;
      }
      split.push_back(name.substr(prev_pos, pos));
    }

    if (split.size() == 1) {
      return lookupUnqualifiedElement(split.front());
    } else {
      auto top_scope = lookupUnqualifiedScope(split.front());
      if (top_scope) {
        return lookupQualifiedElement(top_scope, split.begin() + 1, split.end());
      } else {
        return Element{};
      }
    }
  }

  auto getQualifiedName() const -> std::string
  {
    std::list<const EnvironmentFrame *> ancestors;
    for (auto * p = this; p != nullptr; p = p->parent) {
      ancestors.push_back(p);
    }
    std::string ret;
    for (auto it = ancestors.rbegin(); it != ancestors.rend(); ++it) {
      ret += (it == ancestors.rbegin() ? "" : "::");
      ret += (*it)->scope_name.empty() ? "{anonymous}" : (*it)->scope_name;
    }
    return ret;
  }

private:
  auto lookupChildElement(const std::string & name) const -> Element
  {
    std::vector<const EnvironmentFrame *> same_level{this};

    while (not same_level.empty()) {
      std::vector<const EnvironmentFrame *> next_level;
      std::vector<Element> ret;

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
    return Element{};
  }

  auto lookupUnqualifiedElement(const std::string & name) const -> Element
  {
    for (auto * p = this; p != nullptr; p = p->parent) {
      auto found = p->lookupChildElement(name);
      if (found) {
        return found;
      }
    }
    return Element{};
  }

  template <typename Iterator>
  static auto lookupQualifiedElement(
    const EnvironmentFrame * scope, Iterator name_begin, Iterator name_end) -> Element
  {
    for (auto iter = name_begin; iter != name_end - 1; ++iter) {
      auto found = scope->lookupChildScope(*iter);
      if (found.size() == 1) {
        scope = found.front();
      } else if (found.empty()) {
        return Element{};
      } else if (found.size() > 1) {
        THROW_SYNTAX_ERROR("ambiguous reference to ", std::quoted(*iter));
      }
    }

    return scope->lookupChildElement(*(name_end - 1));
  }

  auto lookupChildScope(const std::string & name) const -> std::list<const EnvironmentFrame *>
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

  auto lookupUnqualifiedScope(const std::string & name) const -> const EnvironmentFrame *
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
};

class Scope
{
  const std::shared_ptr<EnvironmentFrame> frame;

  struct GlobalEnvironment
  {
    const boost::filesystem::path pathname;  // for substitution syntax '$(dirname)'

    std::unordered_map<std::string, Element> entities;  // ScenarioObject or EntitySelection

    explicit GlobalEnvironment(const boost::filesystem::path pathname) : pathname(pathname) {}

    auto isAddedEntity(const EntityRef &) const -> bool;
  };

  const std::shared_ptr<GlobalEnvironment> global_environment;

public:
  const std::string name;

  std::list<EntityRef> actors;

  explicit Scope() = delete;

  explicit Scope(const boost::filesystem::path & pathname)
  : frame(new EnvironmentFrame()), global_environment(std::make_shared<GlobalEnvironment>(pathname))
  {
  }

private:
  explicit Scope(
    const Scope & parent, const std::string & name, const std::shared_ptr<EnvironmentFrame> & frame)
  : frame(frame), global_environment(parent.global_environment), name(name), actors(parent.actors)
  {
  }

public:
  Scope(const Scope &) = default;  // NOTE: shallow copy

  Scope(Scope &&) noexcept = default;

  auto global() const -> const auto & { return *global_environment; }

  auto global() -> auto & { return *global_environment; }

  auto localScope() const noexcept -> const auto & { return *this; }

  auto localScope() noexcept -> auto & { return *this; }

  auto makeChildScope(const std::string & name) const
  {
    return Scope(
      *this, name, std::shared_ptr<EnvironmentFrame>(new EnvironmentFrame(*frame, name)));
  }

  auto insert(const std::string & name_, const Element & element)
  {
    return frame->insert(name_, element);
  }

  auto findElement(const std::string & name_) const { return frame->findElement(name_); }
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SCOPE_HPP_
