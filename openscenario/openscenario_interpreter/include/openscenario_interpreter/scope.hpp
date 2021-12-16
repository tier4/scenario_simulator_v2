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

  auto find(const Name &) const -> Object;

  auto find(const Prefixed<Name> &) const -> Object;

  auto findObject(const Prefixed<Name> &) const -> Object;

  auto isOutermost() const noexcept -> bool;

private:
  auto frames(const Name &) const -> std::list<const EnvironmentFrame *>;

  auto lookdown(const std::string &) const -> Object;

  auto lookupFrame(const Name &) const -> const EnvironmentFrame *;

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

  auto findObject(const std::string &) const -> Object;

  auto global() const -> const GlobalEnvironment &;

  auto global() -> GlobalEnvironment &;

  auto local() const noexcept -> const Scope &;

  auto local() noexcept -> Scope &;

  auto insert(const Name &, const Object &) -> void;
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SCOPE_HPP_
