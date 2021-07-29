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
#include <boost/any.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <queue>
#include <scenario_simulator_exception/exception.hpp>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace openscenario_interpreter
{
struct ScopeImpl
{
private:
  friend struct Scope;

  const std::string scope_name;

  std::unordered_map<std::string, Element> enviroments;

  ScopeImpl * parent = nullptr;

  std::unordered_map<std::string, ScopeImpl *> named_children;

  std::vector<ScopeImpl *> anonymous_children;

  ScopeImpl() = default;

  ScopeImpl(ScopeImpl & parent, const std::string & name) : scope_name(name), parent(&parent)
  {
    if (name.empty() || name.find("anonymous") == 0) {
      parent.anonymous_children.push_back(this);
    } else {
      auto ret = parent.named_children.insert({name, this});
      if (!ret.second) {
        THROW_SYNTAX_ERROR("'", name, "' is duplicated in this scope");
      }
    }
  }

  ScopeImpl(const ScopeImpl &) = delete;
  ScopeImpl(ScopeImpl &&) = delete;

public:
  auto addElement(const std::string & name, Element element) -> void
  {
    if (std::any_of(name.begin(), name.end(), boost::is_any_of(": ,."))) {
      THROW_SYNTAX_ERROR("Identifier '", name, "' contains ':', ' ', ',' or '.' ");
    }

    enviroments.insert(std::make_pair(name, std::move(element)));
  }

  auto findElement(const std::string & name) const -> Element
  {
    std::vector<std::string> splitted;
    {
      const char * delim = "::";
      const std::size_t delim_len = 2;

      std::size_t prev_pos = 0, pos = 0;
      while ((pos = name.find(delim, prev_pos)) != std::string::npos) {
        splitted.push_back(name.substr(prev_pos, pos - prev_pos));
        prev_pos = pos + delim_len;
      }
      splitted.push_back(name.substr(prev_pos, pos));
    }

    if (splitted.size() == 1) {
      return lookupUnqualifiedElement(splitted.front());
    } else {
      auto top_scope = lookupUnqualifiedScope(splitted.front());
      return lookupQualifiedElement(top_scope, splitted.begin() + 1, splitted.end());
    }
  }

private:
  auto lookupUnqualifiedElement(const std::string & name) const -> Element
  {
    for (auto * p = this; p != nullptr; p = p->parent) {
      auto found = p->enviroments.find(name);
      if (found != p->enviroments.end()) {
        return found->second;
      }
    }
    return Element{};
  }

  auto lookupChildScope(const std::string & name) const -> std::list<const ScopeImpl *>
  {
    auto found = named_children.find(name);
    if (found != named_children.end()) {
      return {found->second};
    }

    std::list<const ScopeImpl *> ret;
    for (auto & child : anonymous_children) {
      ret.merge(child->lookupChildScope(name));
    }
    return ret;
  }

  auto lookupUnqualifiedScope(const std::string & name) const -> const ScopeImpl *
  {
    if (name.empty() && parent == nullptr) {  // global scope
      return this;
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

  template <typename Iterator>
  auto lookupQualifiedElement(const ScopeImpl * scope, Iterator name_begin, Iterator name_end) const
    -> Element
  {
    for (auto iter = name_begin; iter != name_end - 1; ++iter) {
      auto found = scope->lookupChildScope(*iter);
      if (found.size() == 1) {
        scope = found.front();
      } else if (found.empty()) {
        THROW_SYNTAX_ERROR("undefined reference to ", *iter);
      } else if (found.size() > 1) {
        THROW_SYNTAX_ERROR("ambiguous reference to ", *iter);
      }
    }

    auto found = scope->enviroments.find(*(name_end - 1));
    if (found == scope->enviroments.end()) {
      THROW_SYNTAX_ERROR("undefined reference to ", *(name_end - 1));
    }
    return found->second;
  }
};

struct Scope
{
  using Actor = EntityRef;

  // private:
  const std::shared_ptr<ScopeImpl> impl;

public:
  const std::string name;

  std::list<Actor> actors;

  /* ---- GLOBAL ------------------------------------------------------------ */

  const boost::filesystem::path pathname;  // for substitution syntax '$(dirname)'

  boost::filesystem::path logic_file;  // NOTE: Assigned by RoadNetwork's constructor.

  boost::filesystem::path scene_graph_file;  // NOTE: Assigned by RoadNetwork's constructor.

  Scope() = delete;

  explicit Scope(const boost::filesystem::path & pathname)
  : impl(new ScopeImpl()), pathname(pathname)
  {
  }

private:
  explicit Scope(const Scope & parent, const std::string & name, std::shared_ptr<ScopeImpl> impl_)
  : impl(std::move(impl_)),
    name(name),
    actors(parent.actors),
    pathname(parent.pathname),
    logic_file(parent.logic_file),
    scene_graph_file(parent.scene_graph_file)
  {
  }

public:
  // note: shallow copy
  Scope(const Scope &) = default;
  Scope(Scope &&) = default;

  // clang-format off
  auto localScope()       noexcept ->       auto & { return *this; }
  auto localScope() const noexcept -> const auto & { return *this; }
  // clang-format on

  auto makeChildScope(const std::string & name) const
  {
    return Scope{*this, name, std::shared_ptr<ScopeImpl>(new ScopeImpl(*impl, name))};
  }

  auto addElement(const std::string & name_, const Element & element)
  {
    return impl->addElement(name_, element);
  }

  auto findElement(const std::string & name_) const { return impl->findElement(name_); }
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SCOPE_HPP_
