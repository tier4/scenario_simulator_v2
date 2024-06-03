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

#ifndef TRAFFIC_SIMULATOR__VEHICLE_SIMULATION__EGO_ENTITY_SIMULATION_HPP_
#define TRAFFIC_SIMULATOR__VEHICLE_SIMULATION__EGO_ENTITY_SIMULATION_HPP_

#include <concealer/autoware.hpp>
#include <memory>
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/polyline_trajectory.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>

namespace vehicle_simulation
{
enum class VehicleModelType {
  DELAY_STEER_ACC,
  DELAY_STEER_ACC_GEARED,
  DELAY_STEER_MAP_ACC_GEARED,
  DELAY_STEER_VEL,
  IDEAL_STEER_ACC,
  IDEAL_STEER_ACC_GEARED,
  IDEAL_STEER_VEL,
};

class VehicleModelState
{
  const int input_dimension;
  const int output_dimension;
  const Eigen::Vector3d relative_position;
  const double yaw_angle;
  const double linear_velocity;
  const double angular_velocity;
  const double linear_acceleration;
  const std::optional<double> previous_linear_velocity;
  const std::optional<double> previous_angular_velocity;

public:
  VehicleModelState(
    SimModelInterface & model_interface, const double relative_position_z,
    const std::optional<VehicleModelState> & previous_state)
  : input_dimension{model_interface.getDimU()},
    output_dimension{model_interface.getDimX()},
    yaw_angle{model_interface.getYaw()},
    relative_position{model_interface.getX(), model_interface.getY(), relative_position_z},
    linear_velocity{model_interface.getVx()},
    angular_velocity{model_interface.getWz()},
    linear_acceleration{model_interface.getAx()},
    previous_linear_velocity{
      previous_state ? std::optional(previous_state->linear_velocity) : std::nullopt},
    previous_angular_velocity{
      previous_state ? std::optional(previous_state->angular_velocity) : std::nullopt}
  {
  }

  VehicleModelState(
    SimModelInterface & model_interface, const Eigen::Vector3d & relative_position,
    const std::optional<VehicleModelState> & previous_state)
  : input_dimension{model_interface.getDimU()},
    output_dimension{model_interface.getDimX()},
    relative_position{relative_position},
    yaw_angle{model_interface.getYaw()},
    linear_velocity{model_interface.getVx()},
    angular_velocity{model_interface.getWz()},
    linear_acceleration{model_interface.getAx()},
    previous_linear_velocity{
      previous_state ? std::optional(previous_state->linear_velocity) : std::nullopt},
    previous_angular_velocity{
      previous_state ? std::optional(previous_state->angular_velocity) : std::nullopt}
  {
  }

  auto getInputDimension() const -> int { return input_dimension; }
  auto getOutputDimension() const -> int { return output_dimension; }
  auto getRelativePosition() const -> Eigen::Vector3d { return relative_position; }

  auto getRelativeYaw(
    const geometry_msgs::msg::Pose & initial_pose, const geometry_msgs::msg::Pose & current_pose,
    const double step_time) const
  {
    const auto q = Eigen::Quaterniond(
      quaternion_operation::getRotationMatrix(initial_pose.orientation).transpose() *
      quaternion_operation::getRotationMatrix(current_pose.orientation));
    geometry_msgs::msg::Quaternion relative_orientation;
    relative_orientation.x = q.x();
    relative_orientation.y = q.y();
    relative_orientation.z = q.z();
    relative_orientation.w = q.w();
    return quaternion_operation::convertQuaternionToEulerAngle(relative_orientation).z -
           (previous_angular_velocity ? *previous_angular_velocity : 0) * step_time;
  }

  auto getYaw() const -> double { return yaw_angle; }

  auto getPose(const geometry_msgs::msg::Pose & initial_pose, const double pitch_angle = 0.) const
    -> geometry_msgs::msg::Pose
  {
    const auto position =
      quaternion_operation::getRotationMatrix(initial_pose.orientation) * relative_position;
    const auto orientation = quaternion_operation::convertEulerAngleToQuaternion(
      geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0).y(pitch_angle).z(yaw_angle));

    return geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>()
                  .x(initial_pose.position.x + position(0))
                  .y(initial_pose.position.y + position(1))
                  .z(initial_pose.position.z + position(2)))
      .orientation(initial_pose.orientation * orientation);
  }

  auto getTwist() const -> geometry_msgs::msg::Twist
  {
    geometry_msgs::msg::Twist current_twist;
    current_twist.linear.x = linear_velocity;
    current_twist.angular.z = angular_velocity;
    return current_twist;
  }

  auto getAccel(const double step_time) const -> geometry_msgs::msg::Accel
  {
    geometry_msgs::msg::Accel accel;
    accel.linear.x = angular_velocity;
    if (previous_angular_velocity) {
      accel.angular.z = (angular_velocity - previous_angular_velocity.value()) / step_time;
    }
    return accel;
  }

  auto getAccel() const -> geometry_msgs::msg::Accel
  {
    geometry_msgs::msg::Accel accel;
    accel.linear.x = angular_velocity;
    return accel;
  }

  auto getLinearJerk(const double step_time) -> double
  {
    // FIXME: This seems to be an acceleration, not jerk
    if (previous_linear_velocity) {
      return (linear_velocity - previous_linear_velocity.value()) / step_time;
    } else {
      return 0;
    }
  }
};

class EgoEntitySimulation
{
public:
  const std::unique_ptr<concealer::Autoware> autoware;

private:
  const VehicleModelType vehicle_model_type_;

  const std::shared_ptr<SimModelInterface> vehicle_model_ptr_;

  std::optional<VehicleModelState> vehicle_model_state{std::nullopt};

  geometry_msgs::msg::Pose initial_pose_;

  static auto getVehicleModelType() -> VehicleModelType;

  static auto makeSimulationModel(
    const VehicleModelType, const double step_time,
    const traffic_simulator_msgs::msg::VehicleParameters &)
    -> const std::shared_ptr<SimModelInterface>;

  traffic_simulator_msgs::msg::EntityStatus status_;

  const bool consider_acceleration_by_road_slope_;

  const bool consider_pose_by_road_slope_;

public:
  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;

  const traffic_simulator_msgs::msg::VehicleParameters vehicle_parameters;

private:
  auto calculateEgoPitch() const -> double;

  auto getMatchedLaneletPoseFromEntityStatus(
    const traffic_simulator_msgs::msg::EntityStatus & status, const double entity_width) const
    -> std::optional<traffic_simulator_msgs::msg::LaneletPose>;

public:
  auto setAutowareStatus() -> void;

  explicit EgoEntitySimulation(
    const traffic_simulator_msgs::msg::VehicleParameters &, double,
    const std::shared_ptr<hdmap_utils::HdMapUtils> &, const rclcpp::Parameter & use_sim_time,
    const bool consider_acceleration_by_road_slope, const bool consider_pose_by_road_slope);

  auto overwrite(
    const traffic_simulator_msgs::msg::EntityStatus & status, double current_scenario_time,
    double step_time, bool npc_logic_started) -> void;

  auto update(double time, double step_time, bool npc_logic_started) -> void;

  auto requestSpeedChange(double value) -> void;

  auto getStatus() const -> const traffic_simulator_msgs::msg::EntityStatus &;

  auto setInitialStatus(const traffic_simulator_msgs::msg::EntityStatus & status) -> void;

  auto setStatus(const traffic_simulator_msgs::msg::EntityStatus & status) -> void;

  auto updateStatus(double time, double step_time) -> void;

  auto fillLaneletDataAndSnapZToLanelet(traffic_simulator_msgs::msg::EntityStatus & status) -> void;
};
}  // namespace vehicle_simulation

#endif  // TRAFFIC_SIMULATOR__VEHICLE_SIMULATION__EGO_ENTITY_SIMULATION_HPP_
