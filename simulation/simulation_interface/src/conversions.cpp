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

#include <scenario_simulator_exception/exception.hpp>
#include <simulation_interface/conversions.hpp>
#include <string>
#include <vector>

namespace simulation_interface
{
void toProto(const geometry_msgs::msg::Point & p, geometry_msgs::Point & proto)
{
  proto.set_x(p.x);
  proto.set_y(p.y);
  proto.set_z(p.z);
}

void toMsg(const geometry_msgs::Point & proto, geometry_msgs::msg::Point & p)
{
  p.x = proto.x();
  p.y = proto.y();
  p.z = proto.z();
}

void toProto(const geometry_msgs::msg::Quaternion & q, geometry_msgs::Quaternion & proto)
{
  proto.set_x(q.x);
  proto.set_y(q.y);
  proto.set_z(q.z);
  proto.set_w(q.w);
}

void toMsg(const geometry_msgs::Quaternion & proto, geometry_msgs::msg::Quaternion & q)
{
  q.x = proto.x();
  q.y = proto.y();
  q.z = proto.z();
  q.w = proto.w();
}

void toProto(const geometry_msgs::msg::Pose & p, geometry_msgs::Pose & proto)
{
  toProto(p.position, *proto.mutable_position());
  toProto(p.orientation, *proto.mutable_orientation());
}

void toMsg(const geometry_msgs::Pose & proto, geometry_msgs::msg::Pose & p)
{
  toMsg(proto.position(), p.position);
  toMsg(proto.orientation(), p.orientation);
}

void toProto(const geometry_msgs::msg::Vector3 & v, geometry_msgs::Vector3 & proto)
{
  proto.set_x(v.x);
  proto.set_y(v.y);
  proto.set_z(v.z);
}

void toMsg(const geometry_msgs::Vector3 & proto, geometry_msgs::msg::Vector3 & v)
{
  v.x = proto.x();
  v.y = proto.y();
  v.z = proto.z();
}

void toProto(const geometry_msgs::msg::Twist & t, geometry_msgs::Twist & proto)
{
  toProto(t.linear, *proto.mutable_linear());
  toProto(t.angular, *proto.mutable_angular());
}

void toMsg(const geometry_msgs::Twist & proto, geometry_msgs::msg::Twist & t)
{
  toMsg(proto.linear(), t.linear);
  toMsg(proto.angular(), t.angular);
}

void toProto(const geometry_msgs::msg::Accel & a, geometry_msgs::Accel & proto)
{
  toProto(a.linear, *proto.mutable_linear());
  toProto(a.angular, *proto.mutable_angular());
}

void toMsg(const geometry_msgs::Accel & proto, geometry_msgs::msg::Accel & a)
{
  toMsg(proto.linear(), a.linear);
  toMsg(proto.angular(), a.angular);
}

void toProto(
  const traffic_simulator_msgs::msg::BoundingBox & box, traffic_simulator_msgs::BoundingBox & proto)
{
  toProto(box.center, *proto.mutable_center());
  toProto(box.dimensions, *proto.mutable_dimensions());
}

void toMsg(
  const traffic_simulator_msgs::BoundingBox & proto, traffic_simulator_msgs::msg::BoundingBox & box)
{
  toMsg(proto.center(), box.center);
  toMsg(proto.dimensions(), box.dimensions);
}

void toProto(
  const traffic_simulator_msgs::msg::Performance & performance,
  traffic_simulator_msgs::Performance & proto)
{
  proto.set_max_acceleration(performance.max_acceleration);
  proto.set_max_deceleration(performance.max_deceleration);
  proto.set_max_speed(performance.max_speed);
}

void toMsg(
  const traffic_simulator_msgs::Performance & proto,
  traffic_simulator_msgs::msg::Performance & performance)
{
  performance.max_acceleration = proto.max_acceleration();
  performance.max_deceleration = proto.max_deceleration();
  performance.max_speed = proto.max_speed();
}

void toProto(const traffic_simulator_msgs::msg::Axle & axle, traffic_simulator_msgs::Axle & proto)
{
  proto.set_position_x(axle.position_x);
  proto.set_position_z(axle.position_z);
  proto.set_track_width(axle.track_width);
  proto.set_wheel_diameter(axle.wheel_diameter);
  proto.set_max_steering(axle.max_steering);
}

void toMsg(const traffic_simulator_msgs::Axle & proto, traffic_simulator_msgs::msg::Axle & axle)
{
  axle.position_x = proto.position_x();
  axle.position_z = proto.position_z();
  axle.track_width = proto.track_width();
  axle.wheel_diameter = proto.wheel_diameter();
  axle.max_steering = proto.max_steering();
}

void toProto(
  const traffic_simulator_msgs::msg::Axles & axles, traffic_simulator_msgs::Axles & proto)
{
  toProto(axles.front_axle, *proto.mutable_front_axle());
  toProto(axles.rear_axle, *proto.mutable_rear_axle());
}

void toMsg(const traffic_simulator_msgs::Axles & proto, traffic_simulator_msgs::msg::Axles & axles)
{
  toMsg(proto.front_axle(), axles.front_axle);
  toMsg(proto.rear_axle(), axles.rear_axle);
}

/*
void toProto(
  const traffic_simulator_msgs::msg::Property & p,
  traffic_simulator_msgs::Property & proto)
{
  // proto.set_is_ego(p.is_ego);
}

void toMsg(
  const traffic_simulator_msgs::Property & proto,
  traffic_simulator_msgs::msg::Property & p)
{
  // p.is_ego = proto.is_ego();
}
*/

void toProto(
  const traffic_simulator_msgs::msg::VehicleParameters & p,
  traffic_simulator_msgs::VehicleParameters & proto)
{
  toProto(p.bounding_box, *proto.mutable_bounding_box());
  toProto(p.axles, *proto.mutable_axles());
  toProto(p.performance, *proto.mutable_performance());
  // toProto(p.property, *proto.mutable_property());
  proto.set_name(p.name);
  proto.set_vehicle_category(p.vehicle_category);
}

void toMsg(
  const traffic_simulator_msgs::VehicleParameters & proto,
  traffic_simulator_msgs::msg::VehicleParameters & p)
{
  toMsg(proto.axles(), p.axles);
  toMsg(proto.bounding_box(), p.bounding_box);
  toMsg(proto.performance(), p.performance);
  // toMsg(proto.property(), p.property);
  p.name = proto.name();
  p.vehicle_category = proto.vehicle_category();
}

void toProto(
  const traffic_simulator_msgs::msg::PedestrianParameters & p,
  traffic_simulator_msgs::PedestrianParameters & proto)
{
  toProto(p.bounding_box, *proto.mutable_bounding_box());
  proto.set_name(p.name);
  proto.set_pedestrian_category(p.pedestrian_category);
}

void toMsg(
  const traffic_simulator_msgs::PedestrianParameters & proto,
  traffic_simulator_msgs::msg::PedestrianParameters & p)
{
  p.name = proto.name();
  p.pedestrian_category = proto.pedestrian_category();
  toMsg(proto.bounding_box(), p.bounding_box);
}

void toProto(
  const traffic_simulator_msgs::msg::MiscObjectParameters & p,
  traffic_simulator_msgs::MiscObjectParameters & proto)
{
  toProto(p.bounding_box, *proto.mutable_bounding_box());
  proto.set_name(p.name);
  proto.set_misc_object_category(p.misc_object_category);
}

void toMsg(
  const traffic_simulator_msgs::MiscObjectParameters & proto,
  traffic_simulator_msgs::msg::MiscObjectParameters & p)
{
  p.name = proto.name();
  p.misc_object_category = proto.misc_object_category();
  toMsg(proto.bounding_box(), p.bounding_box);
}

void toProto(
  const traffic_simulator_msgs::msg::ActionStatus & s, traffic_simulator_msgs::ActionStatus & proto)
{
  proto.set_current_action(s.current_action);
  toProto(s.twist, *proto.mutable_twist());
  toProto(s.accel, *proto.mutable_accel());
}

void toMsg(
  const traffic_simulator_msgs::ActionStatus & proto, traffic_simulator_msgs::msg::ActionStatus & s)
{
  s.current_action = proto.current_action();
  toMsg(proto.twist(), s.twist);
  toMsg(proto.accel(), s.accel);
}

void toProto(
  const traffic_simulator_msgs::msg::LaneletPose & pose,
  traffic_simulator_msgs::LaneletPose & proto)
{
  proto.set_lanelet_id(pose.lanelet_id);
  proto.set_s(pose.s);
  proto.set_offset(pose.offset);
  toProto(pose.rpy, *proto.mutable_rpy());
}

void toMsg(
  const traffic_simulator_msgs::LaneletPose & proto,
  traffic_simulator_msgs::msg::LaneletPose & pose)
{
  pose.lanelet_id = proto.lanelet_id();
  pose.s = proto.s();
  pose.offset = proto.offset();
  toMsg(proto.rpy(), pose.rpy);
}

void toProto(
  const traffic_simulator_msgs::msg::EntityType & type, traffic_simulator_msgs::EntityType & proto)
{
  if (type.type == traffic_simulator_msgs::msg::EntityType::EGO) {
    proto = traffic_simulator_msgs::EntityType::EGO;
    return;
  } else if (type.type == traffic_simulator_msgs::msg::EntityType::VEHICLE) {
    proto = traffic_simulator_msgs::EntityType::VEHICLE;
    return;
  } else if (type.type == traffic_simulator_msgs::msg::EntityType::PEDESTRIAN) {
    proto = traffic_simulator_msgs::EntityType::PEDESTRIAN;
    return;
  } else if (type.type == traffic_simulator_msgs::msg::EntityType::MISC_OBJECT) {
    proto = traffic_simulator_msgs::EntityType::MISC_OBJECT;
    return;
  }
  // LCOV_EXCL_START
  std::string message = "type of the Entity Type is invalid!\ntype is " + std::to_string(type.type);
  THROW_SIMULATION_ERROR(message);
  // LCOV_EXCL_STOP
}

void toMsg(
  const traffic_simulator_msgs::EntityType & proto, traffic_simulator_msgs::msg::EntityType & type)
{
  if (proto == traffic_simulator_msgs::EntityType::EGO) {
    type.type = traffic_simulator_msgs::msg::EntityType::EGO;
    return;
  }
  if (proto == traffic_simulator_msgs::EntityType::VEHICLE) {
    type.type = traffic_simulator_msgs::msg::EntityType::VEHICLE;
    return;
  }
  if (proto == traffic_simulator_msgs::EntityType::PEDESTRIAN) {
    type.type = traffic_simulator_msgs::msg::EntityType::PEDESTRIAN;
    return;
  }
  if (proto == traffic_simulator_msgs::EntityType::MISC_OBJECT) {
    type.type = traffic_simulator_msgs::msg::EntityType::MISC_OBJECT;
    return;
  }
  // LCOV_EXCL_START
  std::string message = "type of the Entity Type is invalid!";
  THROW_SIMULATION_ERROR(message);
  // LCOV_EXCL_STOP
}

void toProto(
  const traffic_simulator_msgs::msg::EntityStatus & status,
  traffic_simulator_msgs::EntityStatus & proto)
{
  traffic_simulator_msgs::EntityType type;
  toProto(status.type, type);
  proto.set_type(type);
  proto.set_time(status.time);
  proto.set_name(status.name);
  toProto(status.bounding_box, *proto.mutable_bounding_box());
  toProto(status.action_status, *proto.mutable_action_status());
  toProto(status.pose, *proto.mutable_pose());
  toProto(status.lanelet_pose, *proto.mutable_lanelet_pose());
  proto.set_lanelet_pose_valid(status.lanelet_pose_valid);
}

void toMsg(
  const traffic_simulator_msgs::EntityStatus & proto,
  traffic_simulator_msgs::msg::EntityStatus & status)
{
  traffic_simulator_msgs::msg::EntityType type;
  toMsg(proto.type(), type);
  status.type = type;
  status.time = proto.time();
  status.name = proto.name();
  toMsg(proto.bounding_box(), status.bounding_box);
  toMsg(proto.action_status(), status.action_status);
  toMsg(proto.pose(), status.pose);
  toMsg(proto.lanelet_pose(), status.lanelet_pose);
  status.lanelet_pose_valid = proto.lanelet_pose_valid();
}

void toProto(
  const builtin_interfaces::msg::Duration & duration, builtin_interfaces::Duration & proto)
{
  proto.set_sec(duration.sec);
  proto.set_nanosec(duration.nanosec);
}

void toMsg(const builtin_interfaces::Duration & proto, builtin_interfaces::msg::Duration & duration)
{
  duration.sec = proto.sec();
  duration.nanosec = proto.nanosec();
}

void toProto(const builtin_interfaces::msg::Time & time, builtin_interfaces::Time & proto)
{
  proto.set_sec(time.sec);
  proto.set_nanosec(time.nanosec);
}

void toMsg(const builtin_interfaces::Time & proto, builtin_interfaces::msg::Time & time)
{
  time.sec = proto.sec();
  time.nanosec = proto.nanosec();
}

void toProto(const rosgraph_msgs::msg::Clock & clock, rosgraph_msgs::Clock & proto)
{
  toProto(clock.clock, *proto.mutable_clock());
}

void toMsg(const rosgraph_msgs::Clock & proto, rosgraph_msgs::msg::Clock & clock)
{
  toMsg(proto.clock(), clock.clock);
}

void toProto(const std_msgs::msg::Header & header, std_msgs::Header & proto)
{
  proto.set_frame_id(header.frame_id);
  toProto(header.stamp, *proto.mutable_stamp());
}

void toMsg(const std_msgs::Header & proto, std_msgs::msg::Header & header)
{
  header.frame_id = proto.frame_id();
  toMsg(proto.stamp(), header.stamp);
}

void toProto(
  const autoware_auto_control_msgs::msg::AckermannControlCommand & control_command,
  autoware_control_msgs::ControlCommand & proto)
{
  proto.set_velocity(control_command.longitudinal.speed);
  proto.set_acceleration(control_command.longitudinal.acceleration);
  proto.set_steering_angle(control_command.lateral.steering_tire_angle);
  proto.set_steering_angle_velocity(control_command.lateral.steering_tire_rotation_rate);
}

void toMsg(
  const autoware_control_msgs::ControlCommand & proto,
  autoware_auto_control_msgs::msg::AckermannControlCommand & control_command)
{
  control_command.longitudinal.speed = proto.velocity();
  control_command.longitudinal.acceleration = proto.acceleration();
  control_command.lateral.steering_tire_angle = proto.steering_angle();
  control_command.lateral.steering_tire_rotation_rate = proto.steering_angle_velocity();
}

void toProto(const autoware_auto_vehicle_msgs::msg::GearCommand & shift, autoware_auto_vehicle_msgs::GearCommand & proto)
{
  switch (shift.command) {
    /*
    case autoware_auto_vehicle_msgs::msg::GearCommand::NONE:
      proto.set_data(autoware_auto_vehicle_msgs::GEAR_POSITIONS::NONE);
      break;
    */
    case autoware_auto_vehicle_msgs::msg::GearCommand::PARK:
      proto.set_data(autoware_auto_vehicle_msgs::GEAR_POSITIONS::PARKING);
      break;
    case autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE:
      proto.set_data(autoware_auto_vehicle_msgs::GEAR_POSITIONS::REVERSE);
      break;
      /*
    case autoware_auto_vehicle_msgs::msg::GearCommand::NEUTRAL:
      proto.set_data(autoware_auto_vehicle_msgs::GEAR_POSITIONS::NEUTRAL);
      break;
      */
    case autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE:
      proto.set_data(autoware_auto_vehicle_msgs::GEAR_POSITIONS::DRIVE);
      break;
    case autoware_auto_vehicle_msgs::msg::GearCommand::LOW:
      proto.set_data(autoware_auto_vehicle_msgs::GEAR_POSITIONS::LOW);
      break;
    default:
      THROW_SIMULATION_ERROR(
        "shift position is invalid while converting ROS2 message to proto, shit position is ",
        proto.data());
  }
}

void toMsg(const autoware_auto_vehicle_msgs::GearCommand & proto, autoware_auto_vehicle_msgs::msg::GearCommand & shift)
{
  switch (proto.data()) {
    /*
    case autoware_auto_vehicle_msgs::GEAR_POSITIONS::NONE:
      shift.command = autoware_auto_vehicle_msgs::msg::GearCommand::NONE;
      break;
    */
    case autoware_auto_vehicle_msgs::GEAR_POSITIONS::PARKING:
      shift.command = autoware_auto_vehicle_msgs::msg::GearCommand::PARK;
      break;
    case autoware_auto_vehicle_msgs::GEAR_POSITIONS::REVERSE:
      shift.command = autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE;
      break;
      /*
    case autoware_auto_vehicle_msgs::GEAR_POSITIONS::NEUTRAL:
      shift.command = autoware_auto_vehicle_msgs::msg::GearCommand::NEUTRAL;
      break;
      */
    case autoware_auto_vehicle_msgs::GEAR_POSITIONS::DRIVE:
      shift.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE;
      break;
    case autoware_auto_vehicle_msgs::GEAR_POSITIONS::LOW:
      shift.command = autoware_auto_vehicle_msgs::msg::GearCommand::LOW;
      break;
    default:
      THROW_SIMULATION_ERROR(
        "shift position is invalid while converting proto to ROS2 message, shit position is ",
        proto.data());
  }
}

void toProto(const autoware_auto_vehicle_msgs::msg::GearReport & shift, autoware_auto_vehicle_msgs::GearReport & proto)
{
  switch (shift.report) {
    /*
    case autoware_auto_vehicle_msgs::msg::GearReport::NONE:
      proto.set_data(autoware_auto_vehicle_msgs::GEAR_POSITIONS::NONE);
      break;
    */
    case autoware_auto_vehicle_msgs::msg::GearReport::PARK:
      proto.set_data(autoware_auto_vehicle_msgs::GEAR_POSITIONS::PARKING);
      break;
    case autoware_auto_vehicle_msgs::msg::GearReport::REVERSE:
      proto.set_data(autoware_auto_vehicle_msgs::GEAR_POSITIONS::REVERSE);
      break;
      /*
    case autoware_auto_vehicle_msgs::msg::GearReport::NEUTRAL:
      proto.set_data(autoware_auto_vehicle_msgs::GEAR_POSITIONS::NEUTRAL);
      break;
      */
    case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE:
      proto.set_data(autoware_auto_vehicle_msgs::GEAR_POSITIONS::DRIVE);
      break;
    case autoware_auto_vehicle_msgs::msg::GearReport::LOW:
      proto.set_data(autoware_auto_vehicle_msgs::GEAR_POSITIONS::LOW);
      break;
    default:
      THROW_SIMULATION_ERROR(
        "shift position is invalid while converting ROS2 message to proto, shit position is ",
        proto.data());
  }
}

void toMsg(const autoware_auto_vehicle_msgs::GearReport & proto, autoware_auto_vehicle_msgs::msg::GearReport & shift)
{
  switch (proto.data()) {
    /*
    case autoware_auto_vehicle_msgs::GEAR_POSITIONS::NONE:
      shift.report = autoware_auto_vehicle_msgs::msg::GearReport::NONE;
      break;
    */
    case autoware_auto_vehicle_msgs::GEAR_POSITIONS::PARKING:
      shift.report = autoware_auto_vehicle_msgs::msg::GearReport::PARK;
      break;
    case autoware_auto_vehicle_msgs::GEAR_POSITIONS::REVERSE:
      shift.report = autoware_auto_vehicle_msgs::msg::GearReport::REVERSE;
      break;
      /*
    case autoware_auto_vehicle_msgs::GEAR_POSITIONS::NEUTRAL:
      shift.report = autoware_auto_vehicle_msgs::msg::GearReport::NEUTRAL;
      break;
      */
    case autoware_auto_vehicle_msgs::GEAR_POSITIONS::DRIVE:
      shift.report = autoware_auto_vehicle_msgs::msg::GearReport::DRIVE;
      break;
    case autoware_auto_vehicle_msgs::GEAR_POSITIONS::LOW:
      shift.report = autoware_auto_vehicle_msgs::msg::GearReport::LOW;
      break;
    default:
      THROW_SIMULATION_ERROR(
        "shift position is invalid while converting proto to ROS2 message, shit position is ",
        proto.data());
  }
}

void toProto(
  const autoware_auto_control_msgs::msg::AckermannLateralCommand & message,
  autoware_auto_control_msgs::AckermannLateralCommand & proto)
{
  toProto(message.stamp, *proto.mutable_stamp());
  proto.set_steering_tire_angle(message.steering_tire_angle);
  proto.set_steering_tire_rotation_rate(message.steering_tire_rotation_rate);
}

void toMsg(
  const autoware_auto_control_msgs::AckermannLateralCommand & proto,
  autoware_auto_control_msgs::msg::AckermannLateralCommand & message)
{
  toMsg(proto.stamp(), message.stamp);
  message.steering_tire_angle = proto.steering_tire_angle();
  message.steering_tire_rotation_rate = proto.steering_tire_rotation_rate();
}

void toProto(
  const autoware_auto_control_msgs::msg::LongitudinalCommand & message,
  autoware_auto_control_msgs::LongitudinalCommand & proto)
{
  toProto(message.stamp, *proto.mutable_stamp());
  proto.set_speed(message.speed);
  proto.set_acceleration(message.acceleration);
  proto.set_jerk(message.jerk);
}

void toMsg(
  const autoware_auto_control_msgs::LongitudinalCommand & proto,
  autoware_auto_control_msgs::msg::LongitudinalCommand & message)
{
  toMsg(proto.stamp(), message.stamp);
  message.speed = proto.speed();
  message.acceleration = proto.acceleration();
  message.jerk = proto.jerk();
}

void toProto(
  const autoware_auto_control_msgs::msg::AckermannControlCommand & message,
  autoware_auto_control_msgs::AckermannControlCommand & proto)
{
  toProto(message.stamp, *proto.mutable_stamp());
  toProto(message.lateral, *proto.mutable_lateral());
  toProto(message.longitudinal, *proto.mutable_longitudinal());
}

void toMsg(
  const autoware_auto_control_msgs::AckermannControlCommand & proto,
  autoware_auto_control_msgs::msg::AckermannControlCommand & message)
{
  toMsg(proto.stamp(), message.stamp);
  toMsg(proto.lateral(), message.lateral);
  toMsg(proto.longitudinal(), message.longitudinal);
}

void toProto(
  const autoware_vehicle_msgs::msg::VehicleEmergencyStamped & message,
  autoware_vehicle_msgs::VehicleEmergencyStamped & proto)
{
  toProto(message.stamp, *proto.mutable_stamp());
  proto.set_emergency(message.emergency);
}

void toMsg(
  const autoware_vehicle_msgs::VehicleEmergencyStamped & proto,
  autoware_vehicle_msgs::msg::VehicleEmergencyStamped & message)
{
  toMsg(proto.stamp(), message.stamp);
  message.emergency = proto.emergency();
}

void toProto(
  const autoware_auto_perception_msgs::msg::TrafficSignal & traffic_light_state,
  simulation_api_schema::TrafficLightState & proto)
{
  proto.set_id(traffic_light_state.map_primitive_id);
  for (const autoware_auto_perception_msgs::msg::TrafficLight & ls : traffic_light_state.lights) {
    simulation_api_schema::TrafficLightState::LampState lamp_state;
    // TODO(murooka) type is seperated into color, shape, status
    // lamp_state.set_type((simulation_api_schema::TrafficLightState_LampState_State)ls.type);
    *proto.add_lights() = lamp_state;
  }
}
}  // namespace simulation_interface
