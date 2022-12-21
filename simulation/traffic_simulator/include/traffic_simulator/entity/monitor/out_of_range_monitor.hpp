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

#ifndef TRAFFIC_SIMULATOR__ENTITY__MONITOR__OUT_OF_RANGE_MONITOR_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__MONITOR__OUT_OF_RANGE_MONITOR_HPP_

#include <optional>
#include <string>
#include <traffic_simulator/entity/entity_base.hpp>

namespace traffic_simulator::entity
{
template <typename ValuePolicy>
class OutOfRangeMonitor : private ValuePolicy
{
public:
  /**
   * @brief Construct a new Out Of Range Monitor object
   * @param entity target entity
   * @param min_value min range value
   * @param max_value max range value
   * @param name range name
   * @note If return value of ValuePolicy::getValue() goes outside of this range, an exception is thrown.
   */
  OutOfRangeMonitor(
    EntityBase & entity, std::string name, std::optional<double> min_value,
    std::optional<double> max_value, ValuePolicy policy = ValuePolicy())
  : ValuePolicy(policy),
    min_value(min_value),
    max_value(max_value),
    name(std::move(name)),
    entity_(entity)
  {
  }

  auto operator()(double) -> bool
  {
    auto value = this->getValue(entity_);
    if ((min_value && value < *min_value) || (max_value && value > *max_value)) {
      auto min_string = min_value ? std::to_string(*min_value) : "(None)";
      auto max_string = max_value ? std::to_string(*max_value) : "(None)";
      throw SPECIFICATION_VIOLATION(
        "current ", name, " (", value, ") is out of range ([", min_string, ", ", max_string, "])");
    }
    return false;
  }

  const std::optional<double> min_value;
  const std::optional<double> max_value;
  const std::string name;

private:
  EntityBase & entity_;
};
}  // namespace traffic_simulator::entity

#endif  // TRAFFIC_SIMULATOR__ENTITY__MONITOR__OUT_OF_RANGE_MONITOR_HPP_
