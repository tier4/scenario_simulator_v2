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

#ifndef TRAFFIC_SIMULATOR__ENTITY__MONITOR__STOP_LINE_DISTANCE_POLICY_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__MONITOR__STOP_LINE_DISTANCE_POLICY_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>

namespace traffic_simulator::entity
{
class StopLineDistancePolicy
{
public:
  using HdMapUtilsPtr = std::shared_ptr<hdmap_utils::HdMapUtils>;

  explicit StopLineDistancePolicy(HdMapUtilsPtr hdmap_utils_ptr);

  auto getDistance(EntityBase & entity, std::int64_t stop_line_id) -> std::optional<double>;

private:
  HdMapUtilsPtr hdmap_utils_ptr_;
};
}  // namespace traffic_simulator::entity

#endif  // TRAFFIC_SIMULATOR__ENTITY__MONITOR__STOP_LINE_DISTANCE_POLICY_HPP_
