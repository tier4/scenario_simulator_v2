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

  const std::string qualifier;

  std::unordered_multimap<std::string, Object> environments;

  EnvironmentFrame * const parent = nullptr;

  std::unordered_multimap<std::string, EnvironmentFrame *> named_children;

  std::vector<EnvironmentFrame *> anonymous_children;

  explicit EnvironmentFrame() = default;

  explicit EnvironmentFrame(EnvironmentFrame &, const std::string &);

public:
  explicit EnvironmentFrame(const EnvironmentFrame &) = delete;

  explicit EnvironmentFrame(EnvironmentFrame &&) = delete;

  auto define(const Name &, const Object &) -> void;

  auto findObject(const PrefixedName &) const -> Object;

  [[deprecated]] auto fullyQualifiedName() const -> std::string;

private:
  auto lookupChildElement(const std::string &) const -> Object;

  auto lookupChildScope(const std::string &) const -> std::list<const EnvironmentFrame *>;

  static auto lookupQualifiedElement(
    const EnvironmentFrame *,                  //
    std::vector<std::string>::const_iterator,  //
    std::vector<std::string>::const_iterator) -> Object;

  auto lookup(const Name &) const -> Object;

  auto lookupUnqualifiedScope(const std::string &) const -> const EnvironmentFrame *;
};

class Scope
{
  struct GlobalEnvironment
  {
    const boost::filesystem::path pathname;  // for substitution syntax '$(dirname)'

    std::unordered_map<std::string, Object> entities;  // ScenarioObject or EntitySelection

    const CatalogLocations * catalog_locations;

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

public:
  auto findObject(const std::string &) const -> Object;

  auto global() const -> const GlobalEnvironment &;

  auto global() -> GlobalEnvironment &;

  auto local() const noexcept -> const Scope &;

  auto local() noexcept -> Scope &;

  auto insert(const Name &, const Object &) -> void;
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SCOPE_HPP_
