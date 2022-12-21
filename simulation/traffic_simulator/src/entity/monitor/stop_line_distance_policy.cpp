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

#include <geometry/spline/catmull_rom_spline.hpp>
#include <limits>
#include <optional>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/entity/monitor/stop_line_ditance_policy.hpp>
#include <utility>

namespace traffic_simulator::entity
{
StopLineDistancePolicy::StopLineDistancePolicy(HdMapUtilsPtr hdmap_utils_ptr)
: hdmap_utils_ptr_(std::move(hdmap_utils_ptr))
{
}

auto StopLineDistancePolicy::getDistance(EntityBase & entity, std::int64_t stop_line_id)
  -> std::optional<double>
{
  auto waypoints = entity.getWaypoints().waypoints;
  if (waypoints.empty()) {
    return std::nullopt;
  }

  auto spline = math::geometry::CatmullRomSpline(waypoints);
  auto polygon = hdmap_utils_ptr_->getStopLinePolygon(stop_line_id);
  return spline.getCollisionPointIn2D(polygon).value();
}

}  // namespace traffic_simulator::entity
