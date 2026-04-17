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

#ifndef OSI_INTERFACE__ENTITY_ID_REGISTRY_HPP_
#define OSI_INTERFACE__ENTITY_ID_REGISTRY_HPP_

#include <osi3/osi_common.pb.h>

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

namespace osi_interface
{
class EntityIdRegistry
{
public:
  EntityIdRegistry() = default;

  auto assign(const std::string & name) -> osi3::Identifier;

  // Register a name with a specific ID received from an external source (e.g., OSI message).
  auto insertMapping(const std::string & name, uint64_t id) -> void;

  auto lookup(const std::string & name) const -> std::optional<osi3::Identifier>;

  auto reverseLookup(uint64_t id) const -> std::optional<std::string>;

  auto remove(const std::string & name) -> bool;

  auto clear() -> void;

  auto size() const -> std::size_t;

private:
  uint64_t next_id_{1};  // 0 is reserved as invalid
  std::unordered_map<std::string, uint64_t> name_to_id_;
  std::unordered_map<uint64_t, std::string> id_to_name_;
};

}  // namespace osi_interface

#endif  // OSI_INTERFACE__ENTITY_ID_REGISTRY_HPP_
