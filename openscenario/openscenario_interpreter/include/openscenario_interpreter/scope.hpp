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

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <memory>
#include <openscenario_interpreter/name.hpp>
#include <openscenario_interpreter/syntax/catalog_locations.hpp>
#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

namespace openscenario_interpreter
{
class EnvironmentFrame
{
  friend struct Scope;

  std::unordered_multimap<std::string, Object> variables;

  EnvironmentFrame * const outer_frame = nullptr;

  std::unordered_multimap<std::string, EnvironmentFrame *> inner_frames;

  std::vector<EnvironmentFrame *> unnamed_inner_frames;

  explicit EnvironmentFrame() = default;

  explicit EnvironmentFrame(EnvironmentFrame &, const std::string &);

public:
  explicit EnvironmentFrame(const EnvironmentFrame &) = delete;

  explicit EnvironmentFrame(EnvironmentFrame &&) = delete;

  auto define(const Name &, const Object &) -> void;

  template <typename T>
  auto find(const Name & name) const -> Object
  {
    for (auto frame = this; frame; frame = frame->outer_frame) {
      auto object = frame->lookdown(name);
      if (object) {
        return object;
      }
    }

    throw SyntaxError("No such variable ", std::quoted(name));
  }

  template <typename T>
  auto find(const Prefixed<Name> & prefixed_name) const -> Object
  {
    if (not prefixed_name.prefixes.empty()) {
      auto found = frames(prefixed_name);
      switch (found.size()) {
        case 0:
          throw SyntaxError(
            "No such variable ", std::quoted(boost::lexical_cast<std::string>(prefixed_name)), ".");
        case 1:
          return found.front()->find<T>(prefixed_name.strip<1>());
        default:
          throw SyntaxError(
            "Ambiguous reference to ", std::quoted(boost::lexical_cast<std::string>(prefixed_name)),
            ".");
      }
    } else {
      return lookdown(prefixed_name.name);
    }
  }

  template <typename T>
  auto ref(const Prefixed<Name> & prefixed_name) const -> Object
  {
    if (prefixed_name.absolute) {
      return outermostFrame().find<T>(prefixed_name);
    } else if (prefixed_name.prefixes.empty()) {
      return find<T>(prefixed_name.name);
    } else {
      return lookupFrame(prefixed_name)->find<T>(prefixed_name.strip<1>());
    }
  }

  auto isOutermost() const noexcept -> bool;

private:
  auto frames(const Prefixed<Name> &) const -> std::list<const EnvironmentFrame *>;

  auto lookdown(const std::string &) const -> Object;

  auto lookupFrame(const Prefixed<Name> &) const -> const EnvironmentFrame *;

  auto outermostFrame() const noexcept -> const EnvironmentFrame &;
};

class Scope
{
  struct GlobalEnvironment
  {
    const boost::filesystem::path pathname;  // for substitution syntax '$(dirname)'

    std::unordered_map<std::string, Object> entities;  // ScenarioObject or EntitySelection

    const CatalogLocations * catalog_locations = nullptr;

    explicit GlobalEnvironment(const boost::filesystem::path &);

    auto entityRef(const EntityRef &) const -> Object;  // TODO: RETURN ScenarioObject TYPE!

    auto isAddedEntity(const EntityRef &) const -> bool;
  };

  const std::shared_ptr<EnvironmentFrame> frame;

  const std::shared_ptr<GlobalEnvironment> global_environment;

public:
  const std::string name;

  std::list<EntityRef> actors;

  Scope() = delete;

  Scope(const Scope &) = default;  // NOTE: shallow copy

  Scope(Scope &&) noexcept = default;

  explicit Scope(const std::string &, const Scope &);

  explicit Scope(const boost::filesystem::path &);

  template <typename... Ts>
  auto ref(Ts &&... xs) const -> decltype(auto)
  {
    return frame->ref<Object>(std::forward<decltype(xs)>(xs)...);
  }

  auto global() const -> const GlobalEnvironment &;

  auto global() -> GlobalEnvironment &;

  auto local() const noexcept -> const Scope &;

  auto local() noexcept -> Scope &;

  auto insert(const Name &, const Object &) -> void;
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SCOPE_HPP_
