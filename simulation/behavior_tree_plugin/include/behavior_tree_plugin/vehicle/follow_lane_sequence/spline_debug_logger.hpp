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

#ifndef BEHAVIOR_TREE_PLUGIN__VEHICLE__FOLLOW_LANE_SEQUENCE__SPLINE_DEBUG_LOGGER_HPP_
#define BEHAVIOR_TREE_PLUGIN__VEHICLE__FOLLOW_LANE_SEQUENCE__SPLINE_DEBUG_LOGGER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace entity_behavior::vehicle::follow_lane_sequence
{
auto logSplineDebugInfo(
  const std::string & action_name,
  const traffic_simulator_msgs::msg::WaypointsArray & waypoints,
  const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> &
    canonicalized_entity_status) -> std::vector<visualization_msgs::msg::Marker>;
}  // namespace entity_behavior::vehicle::follow_lane_sequence

#endif  // BEHAVIOR_TREE_PLUGIN__VEHICLE__FOLLOW_LANE_SEQUENCE__SPLINE_DEBUG_LOGGER_HPP_
