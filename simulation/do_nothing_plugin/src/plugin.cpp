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
#include <geometry/quaternion/get_rotation.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/quaternion/slerp.hpp>
#include <geometry/vector3/hypot.hpp>
#include <geometry/vector3/operator.hpp>

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
  using math::geometry::operator*;
  using math::geometry::operator-;
  using math::geometry::operator+;

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
        .orientation(math::geometry::slerp(
          v0.position.orientation, v1.position.orientation, interpolation_ratio));
    const double linear_velocity =
      math::geometry::hypot(v1.position.position, v0.position.position) / (v1.time - v0.time);
    const auto linear_acceleration =
      (entity_status->getTwist().linear.x - linear_velocity) / (v1.time - v0.time);
    const auto linear_jerk =
      (entity_status->getAccel().linear.x - linear_acceleration) / (v1.time - v0.time);

    const double angular_velocity =
      math::geometry::convertQuaternionToEulerAngle(
        math::geometry::getRotation(v0.position.orientation, v1.position.orientation))
        .z /
      (v1.time - v0.time);
    const auto angular_acceleration =
      (entity_status->getTwist().angular.x - angular_velocity) / (v1.time - v0.time);

    interpolated_entity_status.action_status.twist =
      geometry_msgs::build<geometry_msgs::msg::Twist>()
        .linear(geometry_msgs::build<geometry_msgs::msg::Vector3>().x(linear_velocity).y(0).z(0))
        .angular(geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0).y(0).z(angular_velocity));
    interpolated_entity_status.action_status.accel =
      geometry_msgs::build<geometry_msgs::msg::Accel>()
        .linear(
          geometry_msgs::build<geometry_msgs::msg::Vector3>().x(linear_acceleration).y(0).z(0))
        .angular(
          geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0).y(0).z(angular_acceleration));
    interpolated_entity_status.action_status.linear_jerk = linear_jerk;
    return interpolated_entity_status;
  };

  if (
    (current_time + step_time) <=
    (trajectory->base_time + trajectory->shape.vertices.begin()->time)) {
    return std::nullopt;
  }
  if (
    (trajectory->base_time + trajectory->shape.vertices.back().time) <=
    (current_time + step_time)) {
    return interpolate_entity_status(
      1, *std::prev(trajectory->shape.vertices.end(), 2),
      *std::prev(trajectory->shape.vertices.end(), 1));
  }
  if (const auto vertex_iter = std::adjacent_find(
        trajectory->shape.vertices.begin(), trajectory->shape.vertices.end(),
        [&](const auto & vertex_a, const auto & vertex_b) {
          return (trajectory->base_time + vertex_a.time) <= (current_time + step_time) and
                 (current_time + step_time) <= (trajectory->base_time + vertex_b.time);
        });
      vertex_iter != trajectory->shape.vertices.end()) {
    return interpolate_entity_status(
      (current_time + step_time - trajectory->base_time - vertex_iter->time) /
        (std::next(vertex_iter)->time - vertex_iter->time),
      *vertex_iter, *std::next(vertex_iter));
  } else {
    return std::nullopt;
  }
}
}  // namespace follow_trajectory
}  // namespace do_nothing_behavior

void DoNothingBehavior::configure(const rclcpp::Logger &) {}

void DoNothingBehavior::update(double current_time, double step_time)
{
  setCurrentTime(current_time);
  setStepTime(step_time);

  const auto interpolate_entity_status_on_polyline_trajectory = [&]() {
    do_nothing_behavior::follow_trajectory::checkPolylineTrajectory(getPolylineTrajectory());
    if (
      const auto interpolated_status =
        do_nothing_behavior::follow_trajectory::interpolateEntityStatusFromPolylineTrajectory(
          getPolylineTrajectory(), getCanonicalizedEntityStatus(), getCurrentTime(),
          getStepTime())) {
      return interpolated_status.value();
    } else {
      return static_cast<traffic_simulator::EntityStatus>(*canonicalized_entity_status_);
    }
  };

  canonicalized_entity_status_->setTime(current_time);
  if (getRequest() == traffic_simulator::behavior::Request::FOLLOW_POLYLINE_TRAJECTORY) {
    canonicalized_entity_status_->set(
      interpolate_entity_status_on_polyline_trajectory(),
      getDefaultMatchingDistanceForLaneletPoseCalculation(), getHdMapUtils());
    if (
      getCurrentTime() + getStepTime() >=
      do_nothing_behavior::follow_trajectory::getLastVertexTimestamp(getPolylineTrajectory())) {
      setRequest(traffic_simulator::behavior::Request::NONE);
    }
  } else {
    canonicalized_entity_status_->set(
      static_cast<traffic_simulator::EntityStatus>(*canonicalized_entity_status_),
      getDefaultMatchingDistanceForLaneletPoseCalculation(), getHdMapUtils());
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
