// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#include <osi_interface/entity_id_registry.hpp>

namespace osi_interface
{
auto EntityIdRegistry::assign(const std::string & name) -> osi3::Identifier
{
  if (auto it = name_to_id_.find(name); it != name_to_id_.end()) {
    osi3::Identifier id;
    id.set_value(it->second);
    return id;
  }

  const auto id_value = next_id_++;
  name_to_id_[name] = id_value;
  id_to_name_[id_value] = name;

  osi3::Identifier id;
  id.set_value(id_value);
  return id;
}

auto EntityIdRegistry::insertMapping(const std::string & name, uint64_t id) -> void
{
  name_to_id_[name] = id;
  id_to_name_[id] = name;
  if (id >= next_id_) {
    next_id_ = id + 1;
  }
}

auto EntityIdRegistry::lookup(const std::string & name) const -> std::optional<osi3::Identifier>
{
  if (auto it = name_to_id_.find(name); it != name_to_id_.end()) {
    osi3::Identifier id;
    id.set_value(it->second);
    return id;
  }
  return std::nullopt;
}

auto EntityIdRegistry::reverseLookup(uint64_t id) const -> std::optional<std::string>
{
  if (auto it = id_to_name_.find(id); it != id_to_name_.end()) {
    return it->second;
  }
  return std::nullopt;
}

auto EntityIdRegistry::remove(const std::string & name) -> bool
{
  auto it = name_to_id_.find(name);
  if (it == name_to_id_.end()) {
    return false;
  }
  id_to_name_.erase(it->second);
  name_to_id_.erase(it);
  return true;
}

auto EntityIdRegistry::clear() -> void
{
  name_to_id_.clear();
  id_to_name_.clear();
  next_id_ = 1;
}

auto EntityIdRegistry::size() const -> std::size_t { return name_to_id_.size(); }

}  // namespace osi_interface
