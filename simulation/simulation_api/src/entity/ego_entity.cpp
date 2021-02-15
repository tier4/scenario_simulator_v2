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
#include <quaternion_operation/quaternion_operation.h>
#include <simulation_api/entity/ego_entity.hpp>

#include <string>
#include <memory>

namespace simulation_api
{
namespace entity
{
autoware_auto_msgs::msg::Complex32 EgoEntity::toHeading(const double yaw)
{
  autoware_auto_msgs::msg::Complex32 heading;
  heading.real = static_cast<decltype(heading.real)>(std::cos(yaw * 0.5));
  heading.imag = static_cast<decltype(heading.imag)>(std::sin(yaw * 0.5));
  return heading;
}

bool EgoEntity::setStatus(const openscenario_msgs::msg::EntityStatus & status)
{
  const double wheelbase =
    parameters.axles.front_axle.position_x - parameters.axles.rear_axle.position_x;

  vehicle_model_ptr_ = std::make_shared<SimModelIdealSteerVel>(wheelbase);

  // NOTE Currently, setStatus always succeeds.
  const bool success = VehicleEntity::setStatus(status);

  const auto current_entity_status = getStatus();

  static auto first_time = true;

  if (first_time) {
    waitForAutowareToBeReady();
    std::atomic_load(&autoware)->setInitialPose(current_entity_status.pose);
    std::atomic_load(&autoware)->setInitialTwist();
    updateAutoware(current_entity_status.pose);
    first_time = false;
  }

  autoware_auto_msgs::msg::VehicleKinematicState state;
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

  current_kinematic_state_ = state;
  origin_ = current_entity_status.pose;

  return success;
}

void EgoEntity::onUpdate(double current_time, double step_time)
{
  Eigen::VectorXd input(2);
  {
    input <<
      std::atomic_load(&autoware)->getVehicleCommand().control.velocity,
      std::atomic_load(&autoware)->getVehicleCommand().control.steering_angle;
  }

  std::cout << "input = [" << input.x() << ", " << input.y() << "]" << std::endl;

  vehicle_model_ptr_->setInput(input);
  vehicle_model_ptr_->update(step_time);

  const auto current_entity_status = getEntityStatus(current_time + step_time, step_time);

  updateAutoware(
    current_entity_status.pose,
    (*vehicle_model_ptr_).getVx(),
    (*vehicle_model_ptr_).getWz());

  setStatus(current_entity_status);

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
  rpy.x = 0;
  rpy.y = 0;
  rpy.z = vehicle_model_ptr_->getYaw();

  geometry_msgs::msg::Pose pose;
  pose.position.x = vehicle_model_ptr_->getX();
  pose.position.y = vehicle_model_ptr_->getY();
  std::cout << "x = " << pose.position.x << std::endl;
  std::cout << "y = " << pose.position.y << std::endl;
  pose.position.z = 0.0;
  pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(rpy);

  geometry_msgs::msg::Twist twist;
  twist.linear.x = vehicle_model_ptr_->getVx();
  twist.angular.z = vehicle_model_ptr_->getWz();

  openscenario_msgs::msg::EntityStatus status;
  status.time = time;
  status.type.type = openscenario_msgs::msg::EntityType::EGO;
  status.bounding_box = getBoundingBox();
  status.action_status.twist = twist;

  geometry_msgs::msg::Accel accel;
  if (previous_angular_velocity_ && previous_velocity_) {
    accel.linear.x = (twist.linear.x - previous_velocity_.get()) / step_time;
    accel.angular.z = (twist.angular.z - previous_angular_velocity_.get()) / step_time;
  }
  status.action_status.accel = accel;

  auto rotation_mat = quaternion_operation::getRotationMatrix(origin_.get().orientation);
  Eigen::VectorXd v(3);
  v(0) = pose.position.x;
  v(1) = pose.position.y;
  v(2) = pose.position.z;
  std::cout << "v = " << v << std::endl;
  v = rotation_mat * v;
  std::cout << "v' = " << v << std::endl;
  std::cout << "origin.pose = [\n"
            << "  pose.position.x = " << origin_.get().position.x << "\n"
            << "  pose.position.y = " << origin_.get().position.y << "\n"
            << "  pose.position.z = " << origin_.get().position.z << "\n"
            << "]" << std::endl;;
  status.pose.position.x = v(0) + origin_.get().position.x;
  status.pose.position.y = v(1) + origin_.get().position.y;
  status.pose.position.z = v(2) + origin_.get().position.z;
  std::cout << "status.pose = [\n"
            << "  pose.position.x = " << status.pose.position.x << "\n"
            << "  pose.position.y = " << status.pose.position.y << "\n"
            << "  pose.position.z = " << status.pose.position.z << "\n"
            << "]" << std::endl;;

  status.pose.orientation = origin_.get().orientation * pose.orientation;

  const auto lanelet_pose = hdmap_utils_ptr_->toLaneletPose(status.pose);

  if (lanelet_pose) {
    status.lanelet_pose = lanelet_pose.get();
  } else {
    status.lanelet_pose_valid = false;
  }

  return status;
}
}  // namespace entity
}  // namespace simulation_api
