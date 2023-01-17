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

#ifndef TRAFFIC_SIMULATOR__ENTITY__MONITOR__MOMENTARY_STOP_MONITOR_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__MONITOR__MOMENTARY_STOP_MONITOR_HPP_

#include <cstdint>
#include <traffic_simulator/entity/entity_base.hpp>

namespace traffic_simulator::entity
{
template <typename DistancePolicy>
class MomentaryStopMonitor : private DistancePolicy
{
public:
  /**
   * @brief Construct a new Momentary Stop Monitor object
   *
   * @param entity Name of target entity which you want to check momentary stop.
   * @param min_acceleration Minimum acceleration in stopping sequence.
   * @param max_acceleration Maximum acceleration in stopping sequence.
   * @param stop_target_lanelet_id Lanelet ID of the stop target.
   * @param stop_target_lanelet_type Type of the stop target.
   * @param stop_sequence_start_distance If the entity get closer to the stop target and the distance between target entity
   *  and stop target under the stop_sequence_start_distance, this monitor begins
   * @param stop_sequence_end_distance If the target entity and stop target under this value and the target entity does not stopped,
   * this monitor throws SPECIFICATION_VIOLATION.
   * @param stop_duration If entities stop longer than stop_duration, then this monitor ends successfully.
   */
  MomentaryStopMonitor(
    EntityBase & entity, double min_acceleration, double max_acceleration,
    std::int64_t stop_target_lanelet_id, double stop_sequence_start_distance,
    double stop_sequence_end_distance, double stop_duration,
    DistancePolicy policy = DistancePolicy())
  : DistancePolicy(policy),
    entity_(entity),
    min_acceleration_(min_acceleration),
    max_acceleration_(max_acceleration),
    stop_target_lanelet_id_(stop_target_lanelet_id),
    stop_sequence_start_distance_(stop_sequence_start_distance),
    stop_sequence_end_distance_(stop_sequence_end_distance),
    stop_duration_(stop_duration)
  {
  }

  auto operator()(double) -> bool
  {
    auto distance = this->getDistance(entity_, stop_target_lanelet_id_);
    if (not distance) {
      if (is_target_in_range_) {
        THROW_SIMULATION_ERROR("failed to calculate distance to stop line.");
      }
      return false;
    }
    if (not is_target_in_range_) {
      if (distance.value() > stop_sequence_start_distance_) {
        return false;
      }
      is_target_in_range_ = true;
    }

    double linear_acceleration = entity_.getCurrentAccel().linear.x;
    if (linear_acceleration < min_acceleration_ || linear_acceleration > max_acceleration_) {
      throw SPECIFICATION_VIOLATION("acceleration is out of range");
    }
    if (distance.value() <= stop_sequence_end_distance_) {
      throw SPECIFICATION_VIOLATION("overrun detected");
    }

    return entity_.getStandStillDuration() >= stop_duration_;
  }

private:
  EntityBase & entity_;
  const double min_acceleration_;
  const double max_acceleration_;
  const std::int64_t stop_target_lanelet_id_;
  const double stop_sequence_start_distance_;
  const double stop_sequence_end_distance_;
  const double stop_duration_;
  bool is_target_in_range_ = false;
};
}  // namespace traffic_simulator::entity

#endif  // TRAFFIC_SIMULATOR__ENTITY__MONITOR__MOMENTARY_STOP_MONITOR_HPP_
