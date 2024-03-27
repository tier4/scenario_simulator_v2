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
void DoNothingBehavior::configure(const rclcpp::Logger &) {}

void DoNothingBehavior::update(double current_time, double step_time)
{
  setCurrentTime(current_time);
  setStepTime(step_time);
  entity_status_->setTime(current_time);
  if (getRequest() == traffic_simulator::behavior::Request::FOLLOW_POLYLINE_TRAJECTORY) {
    followPolylineTrajectory();
  } else {
    setUpdatedStatus(entity_status_);
  }
}

void DoNothingBehavior::checkPolylineTrajectory()
{
  if (const auto trajectory = getPolylineTrajectory()) {
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
    if (trajectory->shape.vertices.empty()) {
      THROW_SIMULATION_ERROR(
        "FollowPolylineTrajectory is requested, but trajectory points are empty. Please check "
        "description of the scenario.");
    }
  } else {
    THROW_SIMULATION_ERROR(
      "Traffic simulator send requests of FollowTrajectory, but the trajectory is empty.",
      "This message is not originally intended to be displayed, if you see it, please "
      "contact the developer of traffic_simulator.");
  }
}

void DoNothingBehavior::followPolylineTrajectory()
{
  updated_status_ = std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus>();
  checkPolylineTrajectory();

  if (const auto trajectory = getPolylineTrajectory()) {
    for (size_t i = 0; i < trajectory->shape.vertices.size() - 1; i++) {
      const auto timestamp_i = trajectory->base_time + trajectory->shape.vertices[i].time;
      const auto timestamp_i_1 = trajectory->base_time + trajectory->shape.vertices[i + 1].time;

      const auto interpolate_entity_status = [this](
                                               const double interpolation_ratio,
                                               const traffic_simulator_msgs::msg::Vertex & v0,
                                               const traffic_simulator_msgs::msg::Vertex & v1) {
        auto interpolated_entity_status =
          static_cast<traffic_simulator_msgs::msg::EntityStatus>(*entity_status_);
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
          (entity_status_->getTwist().linear.x - linear_velocity) / (v1.time - v0.time);
        const auto linear_jerk =
          (entity_status_->getAccel().linear.x - linear_acceleration) / (v1.time - v0.time);

        interpolated_entity_status.action_status.twist =
          geometry_msgs::build<geometry_msgs::msg::Twist>()
            .linear(
              geometry_msgs::build<geometry_msgs::msg::Vector3>().x(linear_velocity).y(0).z(0))
            .angular(geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0).y(0).z(0));
        interpolated_entity_status.action_status.accel =
          geometry_msgs::build<geometry_msgs::msg::Accel>()
            .linear(
              geometry_msgs::build<geometry_msgs::msg::Vector3>().x(linear_acceleration).y(0).z(0))
            .angular(geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0).y(0).z(0));
        interpolated_entity_status.action_status.linear_jerk = linear_jerk;
        return interpolated_entity_status;
      };

      if (i == 0 && (current_time_ + step_time_) <= timestamp_i) {
        break;
      }
      if (
        i == (trajectory->shape.vertices.size() - 2) &&
        timestamp_i_1 <= (current_time_ + step_time_)) {
        setUpdatedStatus(std::make_shared<traffic_simulator::CanonicalizedEntityStatus>(
          traffic_simulator::CanonicalizedEntityStatus(
            interpolate_entity_status(
              1, trajectory->shape.vertices[i], trajectory->shape.vertices[i + 1]),
            getHdMapUtils())));
        break;
      }
      if (
        timestamp_i <= (current_time_ + step_time_) &&
        (current_time_ + step_time_) <= timestamp_i_1) {
        setUpdatedStatus(std::make_shared<traffic_simulator::CanonicalizedEntityStatus>(
          traffic_simulator::CanonicalizedEntityStatus(
            interpolate_entity_status(
              (current_time_ + step_time_ - trajectory->base_time - timestamp_i) /
                (timestamp_i_1 - timestamp_i),
              trajectory->shape.vertices[i], trajectory->shape.vertices[i + 1]),
            getHdMapUtils())));
        break;
      }
    }
  }
  if (!updated_status_) {
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
