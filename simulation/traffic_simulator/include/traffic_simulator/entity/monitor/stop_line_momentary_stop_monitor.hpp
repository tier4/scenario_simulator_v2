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

#ifndef TRAFFIC_SIMULATOR__MONITOR__STOP_LINE_MOMENTARY_STOP_MONITOR_HPP_
#define TRAFFIC_SIMULATOR__MONITOR__STOP_LINE_MOMENTARY_STOP_MONITOR_HPP_

#include <optional>
#include <traffic_simulator/entity/monitor/momentary_stop_monitor.hpp>

namespace traffic_simulator::entity
{
class StopLineMomentaryStopMonitor : public MomentaryStopMonitor<StopLineMomentaryStopMonitor>
{
  using Base = MomentaryStopMonitor<StopLineMomentaryStopMonitor>;

public:
  using Base::Base;

  auto getDistance() -> std::optional<double>;
};
}  // namespace traffic_simulator::entity

#endif  // TRAFFIC_SIMULATOR__MONITOR__STOP_LINE_MOMENTARY_STOP_MONITOR_HPP_
