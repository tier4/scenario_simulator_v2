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

#include <arithmetic/floating_point/comparison.hpp>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/get_rotation.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/vector3/hypot.hpp>
#include <geometry/vector3/inner_product.hpp>
#include <geometry/vector3/is_finite.hpp>
#include <geometry/vector3/norm.hpp>
#include <geometry/vector3/operator.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <traffic_simulator/behavior/follow_trajectory/follow_trajectory.hpp>
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator_msgs/msg/action_status.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{

/// @note side effects on polyline_trajectory
auto discardTheFrontWaypoint(
  traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory, const double current_time)
  -> void;

auto makeUpdatedEntityStatus(
  const ValidatedEntityStatus & validated_entity_status,
  traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
  const double matching_distance, const std::optional<double> target_speed, const double step_time,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<EntityStatus>
{
  assert(step_time > 0.0);
  while (not polyline_trajectory.shape.vertices.empty()) {
    const auto updated_entity_opt = PolylineTrajectoryPositioner(
                                      hdmap_utils_ptr, validated_entity_status, polyline_trajectory,
                                      target_speed, matching_distance, step_time)
                                      .makeUpdatedEntityStatus();
    if (updated_entity_opt.has_value()) {
      return updated_entity_opt;
    } else {
      discardTheFrontWaypoint(polyline_trajectory, validated_entity_status.time());
    }
  }
  return std::nullopt;
}

auto discardTheFrontWaypoint(
  traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory, const double current_time)
  -> void
{
  assert(not polyline_trajectory.shape.vertices.empty());
  /*
      The OpenSCENARIO standard does not define the behavior when the value of
      Timing.domainAbsoluteRelative is "relative". The standard only states
      "Definition of time value context as either absolute or relative", and
      it is completely unclear when the relative time starts.

      This implementation has interpreted the specification as follows:
      Relative time starts from the start of FollowTrajectoryAction or from
      the time of reaching the previous "waypoint with arrival time".

      Note: std::isfinite(polyline_trajectory.base_time) means
      "Timing.domainAbsoluteRelative is relative".

      Note: std::isfinite(polyline_trajectory.shape.vertices.front().time)
      means "The waypoint about to be popped is the waypoint with the
      specified arrival time".
  */
  if (
    std::isfinite(polyline_trajectory.base_time) and
    std::isfinite(polyline_trajectory.shape.vertices.front().time)) {
    polyline_trajectory.base_time = current_time;
  }

  std::rotate(
    std::begin(polyline_trajectory.shape.vertices),
    std::begin(polyline_trajectory.shape.vertices) + 1,
    std::end(polyline_trajectory.shape.vertices));

  if (not polyline_trajectory.closed) {
    polyline_trajectory.shape.vertices.pop_back();
  }
};

}  // namespace follow_trajectory
}  // namespace traffic_simulator
