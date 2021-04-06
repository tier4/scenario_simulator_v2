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

#ifndef TRAFFIC_SIMULATOR__METRICS__MOMENTARY_STOP_METRIC_HPP_
#define TRAFFIC_SIMULATOR__METRICS__MOMENTARY_STOP_METRIC_HPP_

#include <traffic_simulator/metrics/metric_base.hpp>

#include <string>

namespace metrics
{
class MomentaryStopMetric : public MetricBase
{
public:
  enum class StopTargetLaneletType
  {
    STOP_LINE,
    CROSSWALK
  };
  MomentaryStopMetric(
    std::string target_entity,
    double min_acceleration,
    double max_acceleration,
    std::int64_t stop_target_lanelet_id,
    StopTargetLaneletType stop_target_lanelet_type,
    double stop_sequence_start_distance,
    double stop_sequence_end_distance,
    double stop_duration)
  : MetricBase("MomentaryStop"),
    target_entity(target_entity),
    min_acceleration(min_acceleration),
    max_acceleration(max_acceleration),
    stop_target_lanelet_id(stop_target_lanelet_id),
    stop_target_lanelet_type(stop_target_lanelet_type),
    stop_sequence_start_distance(stop_sequence_start_distance),
    stop_sequence_end_distance(stop_sequence_end_distance),
    stop_duration(stop_duration) {}

  ~MomentaryStopMetric() override = default;
  void update() override;
  bool activateTrigger() override;
  const std::string target_entity;
  const double min_acceleration;
  const double max_acceleration;
  const std::int64_t stop_target_lanelet_id;
  const StopTargetLaneletType stop_target_lanelet_type;
  const double stop_sequence_start_distance;
  const double stop_sequence_end_distance;
  const double stop_duration;
  nlohmann::json to_json();

private:
  double linear_acceleration_;
  double standstill_duration_;
  double distance_to_stopline_;
};
}  // namespace metrics

#endif  // TRAFFIC_SIMULATOR__METRICS__MOMENTARY_STOP_METRIC_HPP_
