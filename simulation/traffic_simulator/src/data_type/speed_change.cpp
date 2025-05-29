// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include <traffic_simulator/data_type/speed_change.hpp>

namespace traffic_simulator
{
namespace speed_change
{
static_assert(not std::is_default_constructible_v<Constraint>);
static_assert(std::is_copy_constructible_v<Constraint>);
static_assert(std::is_move_constructible_v<Constraint>);
static_assert(std::is_copy_assignable_v<Constraint>);
static_assert(std::is_move_assignable_v<Constraint>);

static_assert(not std::is_default_constructible_v<RelativeTargetSpeed>);
static_assert(std::is_copy_constructible_v<RelativeTargetSpeed>);
static_assert(std::is_move_constructible_v<RelativeTargetSpeed>);
static_assert(std::is_copy_assignable_v<RelativeTargetSpeed>);
static_assert(std::is_move_assignable_v<RelativeTargetSpeed>);

double RelativeTargetSpeed::getAbsoluteValue(
  const CanonicalizedEntityStatus & status,
  const std::unordered_map<std::string, CanonicalizedEntityStatus> & other_status) const
{
  if (const auto iter = other_status.find(reference_entity_name); iter == other_status.end()) {
    if (static_cast<EntityStatus>(status).name == reference_entity_name) {
      switch (type) {
        default:
        case Type::delta:
          return status.getTwist().linear.x + value;
        case Type::factor:
          return status.getTwist().linear.x * value;
      }
    } else {
      THROW_SEMANTIC_ERROR(
        "Reference entity name ", std::quoted(reference_entity_name),
        " is invalid. Please check entity ", std::quoted(reference_entity_name),
        " exists and not a same entity you want to request changing target speed.");
    }

  } else {
    switch (type) {
      default:
      case Type::delta:
        return iter->second.getTwist().linear.x + value;
      case Type::factor:
        return iter->second.getTwist().linear.x * value;
    }
  }
}
}  // namespace speed_change
}  // namespace traffic_simulator
