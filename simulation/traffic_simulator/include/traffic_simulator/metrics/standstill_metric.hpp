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

#ifndef TRAFFIC_SIMULATOR__METRICS__STANDSTILL_METRIC_HPP_
#define TRAFFIC_SIMULATOR__METRICS__STANDSTILL_METRIC_HPP_

#include <boost/optional.hpp>
#include <limits>
#include <string>
#include <traffic_simulator/metrics/metric_base.hpp>

namespace metrics
{
class StandstillMetric : public MetricBase
{
public:
  /**
   * @brief Construct a new Standstill Metric object
   * 
   * @param target_entity name of the target entity
   */
  explicit StandstillMetric(
    std::string target_entity,
    double allow_standstill_duration = std::numeric_limits<double>::max());
  virtual ~StandstillMetric() = default;
  void update() override;
  nlohmann::json toJson();
  bool activateTrigger() override;
  const std::string target_entity;
  const double allow_standstill_duration;

private:
  boost::optional<double> standstill_duration_;
};
}  // namespace metrics

#endif  // TRAFFIC_SIMULATOR__METRICS__STANDSTILL_METRIC_HPP_
