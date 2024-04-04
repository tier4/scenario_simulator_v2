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

#include <do_nothing_plugin/plugin.hpp>
#include <geometry/linear_algebra.hpp>
#include <geometry/vector3/hypot.hpp>

namespace entity_behavior
{
namespace do_nothing_behavior
{
namespace follow_trajectory
{
bool checkPolylineTrajectory(
  const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & trajectory)
{
  if (trajectory) {
    if (trajectory->closed) {
      THROW_SIMULATION_ERROR("Currently, closed trajectory does not supported.");
    }
    if (!trajectory->dynamic_constraints_ignorable) {
      THROW_SIMULATION_ERROR(
        "Currently, dynamic_constraints_ignorable = true (in OpenSCENARIO, followingMode = "
        "follow) does not support in DoNothingBehavior.");
    }
    if (std::abs(trajectory->initial_distance_offset) > std::numeric_limits<double>::epsilon()) {
      THROW_SIMULATION_ERROR(
        "Currently, initial_distance_offset should be 0 when following trajectory in "
        "DoNothingBehavior.",
        "You specified : ", trajectory->initial_distance_offset);
    }
    if (trajectory->shape.vertices.size() <= 1) {
      THROW_SIMULATION_ERROR(
        "FollowPolylineTrajectory is requested, but vertex points are less than 1 point.",
        "At least 2 vertex points are required.", "Please check description of the scenario.");
    }
  } else {
    THROW_SIMULATION_ERROR(
      "Traffic simulator send requests of FollowTrajectory, but the trajectory is empty.",
      "This message is not originally intended to be displayed, if you see it, please "
      "contact the developer of traffic_simulator.");
  }
  return true;
}

auto getLastVertexTimestamp(
  const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & trajectory)
  -> std::optional<double>
{
  checkPolylineTrajectory(trajectory);
  return trajectory->base_time + trajectory->shape.vertices.back().time;
}

auto interpolateEntityStatusFromPolylineTrajectory(
  const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & trajectory,
  const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> & entity_status,
  double current_time, double step_time) -> std::optional<traffic_simulator::EntityStatus>
{
  if (!trajectory) {
    return std::nullopt;
  }

  const auto interpolate_entity_status =
    [&](
      const double interpolation_ratio, const traffic_simulator_msgs::msg::Vertex & v0,
      const traffic_simulator_msgs::msg::Vertex & v1) -> traffic_simulator_msgs::msg::EntityStatus {
    auto interpolated_entity_status =
      static_cast<traffic_simulator_msgs::msg::EntityStatus>(*entity_status);
    interpolated_entity_status.lanelet_pose_valid = false;
    interpolated_entity_status.lanelet_pose = traffic_simulator_msgs::msg::LaneletPose();
    interpolated_entity_status.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(
          v0.position.position * (1 - interpolation_ratio) +
          v1.position.position * interpolation_ratio)
        .orientation(quaternion_operation::slerp(
          v0.position.orientation, v1.position.orientation, interpolation_ratio));
    const double linear_velocity =
      math::geometry::hypot(v1.position.position, v0.position.position) / (v1.time - v0.time);
    const auto linear_acceleration =
      (entity_status->getTwist().linear.x - linear_velocity) / (v1.time - v0.time);
    const auto linear_jerk =
      (entity_status->getAccel().linear.x - linear_acceleration) / (v1.time - v0.time);

    interpolated_entity_status.action_status.twist =
      geometry_msgs::build<geometry_msgs::msg::Twist>()
        .linear(geometry_msgs::build<geometry_msgs::msg::Vector3>().x(linear_velocity).y(0).z(0))
        .angular(geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0).y(0).z(0));
    interpolated_entity_status.action_status.accel =
      geometry_msgs::build<geometry_msgs::msg::Accel>()
        .linear(
          geometry_msgs::build<geometry_msgs::msg::Vector3>().x(linear_acceleration).y(0).z(0))
        .angular(geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0).y(0).z(0));
    interpolated_entity_status.action_status.linear_jerk = linear_jerk;
    return interpolated_entity_status;
  };

  if ((current_time + step_time) <= trajectory->shape.vertices.begin()->time) {
    return std::nullopt;
  }
  if (trajectory->shape.vertices.back().time <= (current_time + step_time)) {
    return interpolate_entity_status(
      1, *std::prev(trajectory->shape.vertices.end(), 2),
      *std::prev(trajectory->shape.vertices.end(), 1));
  }
  for (auto vertex_itr = std::adjacent_find(
         trajectory->shape.vertices.begin(), trajectory->shape.vertices.end(),
         [](const auto &, const auto &) { return true; });
       vertex_itr != trajectory->shape.vertices.end(); vertex_itr++) {
    const auto timestamp_i = trajectory->base_time + vertex_itr->time;
    const auto timestamp_i_1 = trajectory->base_time + (vertex_itr + 1)->time;
    if (timestamp_i <= (current_time + step_time) && (current_time + step_time) <= timestamp_i_1) {
      return interpolate_entity_status(
        (current_time + step_time - trajectory->base_time - timestamp_i) /
          (timestamp_i_1 - timestamp_i),
        *vertex_itr, *std::next(vertex_itr));
    }
  }
  return std::nullopt;
}
}  // namespace follow_trajectory
}  // namespace do_nothing_behavior

void DoNothingBehavior::configure(const rclcpp::Logger &) {}

void DoNothingBehavior::update(double current_time, double step_time)
{
  setCurrentTime(current_time);
  setStepTime(step_time);

  const auto follow_polyline_trajectory = [&]() {
    do_nothing_behavior::follow_trajectory::checkPolylineTrajectory(getPolylineTrajectory());
    if (
      const auto interpolated_status =
        do_nothing_behavior::follow_trajectory::interpolateEntityStatusFromPolylineTrajectory(
          getPolylineTrajectory(), getEntityStatus(), getCurrentTime(), getStepTime())) {
      setUpdatedStatus(std::make_shared<traffic_simulator::CanonicalizedEntityStatus>(
        traffic_simulator::CanonicalizedEntityStatus(
          interpolated_status.value(), getHdMapUtils())));
    } else {
      setUpdatedStatus(entity_status_);
    }
    if (
      getCurrentTime() + getStepTime() >=
      do_nothing_behavior::follow_trajectory::getLastVertexTimestamp(getPolylineTrajectory())) {
      setRequest(traffic_simulator::behavior::Request::NONE);
    }
  };

  entity_status_->setTime(current_time);
  if (getRequest() == traffic_simulator::behavior::Request::FOLLOW_POLYLINE_TRAJECTORY) {
    follow_polyline_trajectory();
  } else {
    setUpdatedStatus(entity_status_);
  }
}

const std::string & DoNothingBehavior::getCurrentAction() const
{
  static const std::string behavior = "do_nothing";
  return behavior;
}
}  // namespace entity_behavior

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(entity_behavior::DoNothingBehavior, entity_behavior::BehaviorPluginBase)
