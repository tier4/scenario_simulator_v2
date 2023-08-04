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
#include <concealer/autoware.hpp>
#include <concealer/field_operator_application.hpp>
#include <memory>
#include <optional>
#include <string>
#include <traffic_simulator/api/configuration.hpp>
#include <traffic_simulator/entity/vehicle_entity.hpp>
#include <traffic_simulator_msgs/msg/entity_type.hpp>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
class EgoEntity : public VehicleEntity
{
  const std::unique_ptr<concealer::FieldOperatorApplication> field_operator_application;

  static auto makeFieldOperatorApplication(const Configuration &)
    -> std::unique_ptr<concealer::FieldOperatorApplication>;

  traffic_simulator_msgs::msg::EntityStatus externally_updated_status_;

public:
  explicit EgoEntity() = delete;

  explicit EgoEntity(
    const std::string & name, const traffic_simulator_msgs::msg::EntityStatus &,
    const traffic_simulator_msgs::msg::VehicleParameters &, const Configuration &);

  explicit EgoEntity(EgoEntity &&) = delete;

  explicit EgoEntity(const EgoEntity &) = delete;

  ~EgoEntity() override = default;

  auto operator=(EgoEntity &&) -> EgoEntity & = delete;

  auto operator=(const EgoEntity &) -> EgoEntity & = delete;

  auto asFieldOperatorApplication() const -> concealer::FieldOperatorApplication & override;

  auto getCurrentAction() const -> std::string override;

  auto getCurrentPose() const -> geometry_msgs::msg::Pose;

  auto getDefaultDynamicConstraints() const
    -> const traffic_simulator_msgs::msg::DynamicConstraints & override;

  auto getBehaviorParameter() const -> traffic_simulator_msgs::msg::BehaviorParameter override;

  auto getEntityTypename() const -> const std::string & override;

  auto getObstacle() -> std::optional<traffic_simulator_msgs::msg::Obstacle> override;

  auto getRouteLanelets(double horizon = 100) const -> std::vector<std::int64_t> override;

  auto getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray override;

  void onUpdate(double current_time, double step_time) override;

  void requestAcquirePosition(const traffic_simulator_msgs::msg::LaneletPose &) override;

  void requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose) override;

  void requestAssignRoute(const std::vector<traffic_simulator_msgs::msg::LaneletPose> &) override;

  void requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> &) override;

  void requestLaneChange(const std::int64_t) override;

  auto requestLaneChange(const traffic_simulator::lane_change::Parameter &) -> void override;

  auto requestSpeedChange(
    const double, const speed_change::Transition, const speed_change::Constraint,
    const bool continuous) -> void override;

  auto requestSpeedChange(
    const speed_change::RelativeTargetSpeed &, const speed_change::Transition,
    const speed_change::Constraint, const bool continuous) -> void override;

  auto setBehaviorParameter(const traffic_simulator_msgs::msg::BehaviorParameter &)
    -> void override;

  auto setStatusExternally(const traffic_simulator_msgs::msg::EntityStatus & status) -> void;

  void requestSpeedChange(double, bool continuous) override;

  void requestSpeedChange(
    const speed_change::RelativeTargetSpeed & target_speed, bool continuous) override;

  auto setVelocityLimit(double) -> void override;

  auto fillLaneletPose(traffic_simulator_msgs::msg::EntityStatus & status) const -> void override;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__EGO_ENTITY_HPP_
