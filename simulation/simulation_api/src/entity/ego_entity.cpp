// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include <simulation_api/entity/ego_entity.hpp>
#include <quaternion_operation/quaternion_operation.h>

#include <string>

namespace simulation_api
{
namespace entity
{
EgoEntity::EgoEntity(
  std::string name, const EntityStatus & initial_state,
  const pugi::xml_node & xml)
: VehicleEntity(name, initial_state, xml)
{
  double wheelbase = parameters.axles.front_axle.position_x -
    parameters.axles.rear_axle.position_x;
  vehicle_model_ptr_ = std::make_shared<SimModelIdealSteerVel>(wheelbase);
  current_kinematic_state_ = boost::none;
}
EgoEntity::EgoEntity(
  std::string name, const EntityStatus & initial_state,
  VehicleParameters parameters)
: VehicleEntity(name, initial_state, parameters)
{
  double wheelbase = parameters.axles.front_axle.position_x -
    parameters.axles.rear_axle.position_x;
  vehicle_model_ptr_ = std::make_shared<SimModelIdealSteerVel>(wheelbase);
  current_kinematic_state_ = boost::none;
}
EgoEntity::EgoEntity(std::string name, const pugi::xml_node & xml)
: VehicleEntity(name, xml)
{
  double wheelbase = parameters.axles.front_axle.position_x -
    parameters.axles.rear_axle.position_x;
  vehicle_model_ptr_ = std::make_shared<SimModelIdealSteerVel>(wheelbase);
  current_kinematic_state_ = boost::none;
}
EgoEntity::EgoEntity(std::string name, VehicleParameters parameters)
: VehicleEntity(name, parameters)
{
  double wheelbase = parameters.axles.front_axle.position_x -
    parameters.axles.rear_axle.position_x;
  vehicle_model_ptr_ = std::make_shared<SimModelIdealSteerVel>(wheelbase);
  current_kinematic_state_ = boost::none;
}
autoware_auto_msgs::msg::Complex32 EgoEntity::toHeading(const double yaw)
{
  autoware_auto_msgs::msg::Complex32 heading;
  heading.real = static_cast<float>(std::cos(yaw * 0.5));
  heading.imag = static_cast<float>(std::sin(yaw * 0.5));
  return heading;
}

bool EgoEntity::setStatus(const EntityStatus & status)
{
  bool ret = VehicleEntity::setStatus(status);
  auto current_entity_status = getStatus();
  autoware_auto_msgs::msg::VehicleKinematicState state;
  state.state.x = current_entity_status.pose.position.x;
  state.state.y = current_entity_status.pose.position.y;
  auto rpy = quaternion_operation::convertQuaternionToEulerAngle(
    current_entity_status.pose.orientation);
  state.state.heading = toHeading(rpy.z);
  state.state.longitudinal_velocity_mps = current_entity_status.twist.linear.x;
  state.state.lateral_velocity_mps = 0;
  state.state.heading_rate_rps = current_entity_status.twist.angular.z;
  state.state.front_wheel_angle_rad = 0;
  state.state.rear_wheel_angle_rad = 0;
  current_kinematic_state_ = state;
  return ret;
}

void EgoEntity::onUpdate(double current_time, double step_time)
{
  if (!status_ || !control_cmd_) {
    return;
  }
  Eigen::VectorXd input(2);
  auto steer = control_cmd_->front_wheel_angle_rad;
  auto acc = control_cmd_->velocity_mps;
  input << acc, steer;
  vehicle_model_ptr_->setInput(input);
  vehicle_model_ptr_->update(step_time);
  status_ = getEntityStatus(current_time + step_time);
}
const EntityStatus EgoEntity::getEntityStatus(double time) const
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = vehicle_model_ptr_->getX();
  pose.position.y = vehicle_model_ptr_->getY();
  pose.position.z = 0.0;
  geometry_msgs::msg::Vector3 rpy;
  rpy.x = 0;
  rpy.y = 0;
  rpy.z = vehicle_model_ptr_->getYaw();
  pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(rpy);
  geometry_msgs::msg::Twist twist;
  twist.linear.x = vehicle_model_ptr_->getVx();
  twist.angular.z = vehicle_model_ptr_->getWz();
  geometry_msgs::msg::Accel accel;
  return EntityStatus(time, pose, twist, accel);
}
void EgoEntity::setVehicleCommands(
  boost::optional<autoware_auto_msgs::msg::VehicleControlCommand> control_cmd,
  boost::optional<autoware_auto_msgs::msg::VehicleStateCommand> state_cmd)
{
  control_cmd_ = control_cmd;
  state_cmd_ = state_cmd;
}
}  // namespace entity
}  // namespace simulation_api
