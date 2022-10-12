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

#ifndef TRAFFIC_SIMULATOR__ENTITY__EGO_ENTITY_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__EGO_ENTITY_HPP_

#include <algorithm>
#include <autoware_auto_system_msgs/msg/emergency_state.hpp>
#include <boost/filesystem.hpp>
#include <concealer/autoware_universe.hpp>
#include <memory>
#include <optional>
#include <string>
#include <traffic_simulator/api/configuration.hpp>
#include <traffic_simulator/entity/vehicle_entity.hpp>
#include <traffic_simulator/vehicle_model/sim_model.hpp>
#include <traffic_simulator/vehicle_model/sim_model_time_delay.hpp>
#include <traffic_simulator_msgs/msg/entity_type.hpp>
#include <vector>

template <typename T>
auto getParameter(const std::string & name, T value = {})
{
  rclcpp::Node node{"get_parameter", "simulation"};

  node.declare_parameter<T>(name, value);
  node.get_parameter<T>(name, value);

  return value;
}

namespace traffic_simulator
{
namespace entity
{
enum class VehicleModelType {
  DELAY_STEER_ACC,
  DELAY_STEER_ACC_GEARED,
  DELAY_STEER_VEL,
  IDEAL_STEER_ACC,
  IDEAL_STEER_ACC_GEARED,
  IDEAL_STEER_VEL,
};

class EgoEntity : public VehicleEntity
{
  const std::unique_ptr<concealer::Autoware> autoware;

  const VehicleModelType vehicle_model_type_;

  const std::shared_ptr<SimModelInterface> vehicle_model_ptr_;

  std::optional<geometry_msgs::msg::Pose> initial_pose_;

  std::optional<double> previous_linear_velocity_, previous_angular_velocity_;

public:
  explicit EgoEntity() = delete;

  explicit EgoEntity(
    const std::string & name,             //
    const Configuration & configuration,  //
    const double step_time,               //
    const traffic_simulator_msgs::msg::VehicleParameters & parameters);

  explicit EgoEntity(EgoEntity &&) = delete;

  explicit EgoEntity(const EgoEntity &) = delete;

  ~EgoEntity() override = default;

  auto operator=(EgoEntity &&) -> EgoEntity & = delete;

  auto operator=(const EgoEntity &) -> EgoEntity & = delete;

  auto asAutoware() const -> concealer::Autoware & override;

  auto getCurrentAction() const -> std::string override;

  auto getCurrentPose() const -> geometry_msgs::msg::Pose;

  auto getCurrentTwist() const -> geometry_msgs::msg::Twist;

  auto getDriverModel() const -> traffic_simulator_msgs::msg::DriverModel override;

  auto getEntityStatus(const double, const double) const
    -> const traffic_simulator_msgs::msg::EntityStatus;

  auto getEntityType() const -> const traffic_simulator_msgs::msg::EntityType & override;

  auto getEntityTypename() const -> const std::string & override;

  auto getObstacle() -> std::optional<traffic_simulator_msgs::msg::Obstacle> override;

  auto getRouteLanelets() const -> std::vector<std::int64_t>;

  auto getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray override;

  void onUpdate(double current_time, double step_time) override;

  void requestAcquirePosition(const traffic_simulator_msgs::msg::LaneletPose &) override;

  void requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose) override;

  void requestAssignRoute(const std::vector<traffic_simulator_msgs::msg::LaneletPose> &) override;

  void requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> &) override;

  void requestLaneChange(const std::int64_t) override;

  auto requestLaneChange(const traffic_simulator::lane_change::Parameter &) -> void override;

  auto requestSpeedChange(
    const double, const speed_change::Transition, const speed_change::Constraint, const bool)
    -> void override;

  auto requestSpeedChange(
    const speed_change::RelativeTargetSpeed &, const speed_change::Transition,
    const speed_change::Constraint, const bool) -> void override;

  auto setDriverModel(const traffic_simulator_msgs::msg::DriverModel &) -> void override;

  auto setStatus(const traffic_simulator_msgs::msg::EntityStatus & status) -> bool override;

  void requestSpeedChange(double, bool) override;

  void requestSpeedChange(
    const speed_change::RelativeTargetSpeed & target_speed, bool continuous) override;

  auto setVelocityLimit(double) -> void override;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__EGO_ENTITY_HPP_
