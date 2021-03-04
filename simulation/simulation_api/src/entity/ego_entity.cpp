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

#include <awapi_accessor/accessor.hpp>
#include <openscenario_msgs/msg/waypoints_array.hpp>
#include <quaternion_operation/quaternion_operation.h>
#include <simulation_api/entity/ego_entity.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace simulation_api
{
namespace entity
{
std::unordered_map<
  std::string, std::shared_ptr<autoware_api::Accessor>
> EgoEntity::autowares {};

autoware_auto_msgs::msg::Complex32 EgoEntity::toHeading(const double yaw)
{
  autoware_auto_msgs::msg::Complex32 heading;
  heading.real = static_cast<decltype(heading.real)>(std::cos(yaw * 0.5));
  heading.imag = static_cast<decltype(heading.imag)>(std::sin(yaw * 0.5));
  return heading;
}

openscenario_msgs::msg::WaypointsArray EgoEntity::getWaypoints() const
{
  openscenario_msgs::msg::WaypointsArray waypoints {};

  for (const auto & point : std::atomic_load(&autowares.at(name))->getTrajectory().points) {
    waypoints.waypoints.emplace_back(point.pose.position);
  }

  return waypoints;
}

bool EgoEntity::setStatus(const openscenario_msgs::msg::EntityStatus & status)
{
  vehicle_model_ptr_ = std::make_shared<SimModelIdealSteerVel>(
    parameters.axles.front_axle.position_x - parameters.axles.rear_axle.position_x);

  // NOTE Currently, setStatus always succeeds.
  const bool success = VehicleEntity::setStatus(status);

  const auto current_entity_status = getStatus();

  if (std::exchange(autoware_uninitialized, false)) {
    std::atomic_load(&autowares.at(name))->setInitialPose(current_entity_status.pose);
    std::atomic_load(&autowares.at(name))->setInitialTwist();

    /* ---- NOTE ---------------------------------------------------------------
     *
     *  awapi_awiv_adapter requires at least 'initialpose' and 'initialtwist'
     *  to be published. Member function EgoEntity::waitForAutowareToBe* are
     *  depends a topic '/awapi/autoware/get/status' published by
     *  awapi_awiv_adapter.
     *
     * ---------------------------------------------------------------------- */
    waitForAutowareStateToBeInitializingVehicle();
  }

  updateAutoware(current_entity_status.pose);

  autoware_auto_msgs::msg::VehicleKinematicState state;
  {
    state.state.x = current_entity_status.pose.position.x;
    state.state.y = current_entity_status.pose.position.y;
    state.state.heading = toHeading(
      quaternion_operation::convertQuaternionToEulerAngle(
        current_entity_status.pose.orientation).z);
    state.state.longitudinal_velocity_mps = current_entity_status.action_status.twist.linear.x;
    state.state.lateral_velocity_mps = 0;
    state.state.heading_rate_rps = current_entity_status.action_status.twist.angular.z;
    state.state.front_wheel_angle_rad = 0;
    state.state.rear_wheel_angle_rad = 0;
  }

  current_kinematic_state_ = state;

  origin_ = current_entity_status.pose;

  return success;
}

void EgoEntity::onUpdate(double current_time, double step_time)
{
  Eigen::VectorXd input(2);
  {
    input <<
      std::atomic_load(&autowares.at(name))->getVehicleCommand().control.velocity,
      std::atomic_load(&autowares.at(name))->getVehicleCommand().control.steering_angle;
  }

  vehicle_model_ptr_->setInput(input);
  vehicle_model_ptr_->update(step_time);

  setStatus(getEntityStatus(current_time + step_time, step_time));

  if (!previous_velocity_) {
    linear_jerk_ = 0;
  } else {
    linear_jerk_ = (vehicle_model_ptr_->getVx() - previous_velocity_.get()) / step_time;
  }

  previous_velocity_ = vehicle_model_ptr_->getVx();
  previous_angular_velocity_ = vehicle_model_ptr_->getWz();
}

const openscenario_msgs::msg::EntityStatus EgoEntity::getEntityStatus(
  const double time,
  const double step_time) const
{
  geometry_msgs::msg::Vector3 rpy;
  {
    rpy.x = 0;
    rpy.y = 0;
    rpy.z = vehicle_model_ptr_->getYaw();
  }

  geometry_msgs::msg::Pose pose;
  {
    pose.position.x = vehicle_model_ptr_->getX();
    pose.position.y = vehicle_model_ptr_->getY();
    pose.position.z = 0.0;
    pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(rpy);
  }

  geometry_msgs::msg::Twist twist;
  {
    twist.linear.x = vehicle_model_ptr_->getVx();
    twist.angular.z = vehicle_model_ptr_->getWz();
  }

  geometry_msgs::msg::Accel accel;
  {
    if (previous_angular_velocity_ && previous_velocity_) {
      accel.linear.x = (twist.linear.x - previous_velocity_.get()) / step_time;
      accel.angular.z = (twist.angular.z - previous_angular_velocity_.get()) / step_time;
    }
  }

  Eigen::VectorXd v(3);
  {
    v(0) = pose.position.x;
    v(1) = pose.position.y;
    v(2) = pose.position.z;
  }

  openscenario_msgs::msg::EntityStatus status;
  {
    const auto rotation_mat = quaternion_operation::getRotationMatrix(origin_.get().orientation);
    v = rotation_mat * v;

    status.time = time;
    status.type.type = openscenario_msgs::msg::EntityType::EGO;
    status.bounding_box = getBoundingBox();
    status.action_status.twist = twist;
    status.action_status.accel = accel;
    status.pose.position.x = v(0) + origin_.get().position.x;
    status.pose.position.y = v(1) + origin_.get().position.y;
    status.pose.position.z = v(2) + origin_.get().position.z;
    status.pose.orientation = origin_.get().orientation * pose.orientation;

    const auto lanelet_pose = hdmap_utils_ptr_->toLaneletPose(status.pose);

    status.lanelet_pose_valid = static_cast<bool>(lanelet_pose);

    if (lanelet_pose) {
      status.lanelet_pose = lanelet_pose.get();
    }
  }

  return status;
}
}  // namespace entity
}  // namespace simulation_api
