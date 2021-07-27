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

#ifndef TRAFFIC_SIMULATOR__ENTITY__EGO_ENTITY_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__EGO_ENTITY_HPP_

#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <concealer/autoware_architecture_proposal.hpp>
#include <memory>
#include <openscenario_msgs/msg/entity_type.hpp>
#include <string>
#include <traffic_simulator/api/configuration.hpp>
#include <traffic_simulator/entity/vehicle_entity.hpp>
#include <traffic_simulator/vehicle_model/sim_model_ideal.hpp>
#include <traffic_simulator/vehicle_model/sim_model_time_delay.hpp>
#include <unordered_map>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
enum class VehicleModelType {
  IDEAL_TWIST = 0,
  IDEAL_STEER = 1,
  DELAY_TWIST = 2,
  DELAY_STEER = 3,
  CONST_ACCEL_TWIST = 4,
  IDEAL_FORKLIFT_RLS = 5,
  DELAY_FORKLIFT_RLS = 6,
  IDEAL_ACCEL = 7,
  DELAY_STEER_ACC = 8,
};

class EgoEntity : public VehicleEntity
{
  // tmp for tests
  // TODO: make it switchable
  concealer::AutowareArchitectureProposal autoware;

  const VehicleModelType vehicle_model_type_;

  const std::shared_ptr<SimModelInterface> vehicle_model_ptr_;

  boost::optional<geometry_msgs::msg::Pose> initial_pose_;

  boost::optional<double> previous_linear_velocity_, previous_angular_velocity_;

public:
  explicit EgoEntity() = delete;

  explicit EgoEntity(
    const std::string & name,             //
    const Configuration & configuration,  //
    const double step_time,               //
    const openscenario_msgs::msg::VehicleParameters & parameters);

  explicit EgoEntity(EgoEntity &&) = delete;

  explicit EgoEntity(const EgoEntity &) = delete;

  ~EgoEntity() override = default;

  auto operator=(EgoEntity &&) -> EgoEntity & = delete;

  auto operator=(const EgoEntity &) -> EgoEntity & = delete;

  void engage() override;

  auto getCurrentAction() const -> const std::string override;

  auto getEntityStatus(const double, const double) const
    -> const openscenario_msgs::msg::EntityStatus;

  auto getEntityTypename() const -> const std::string & override;

  auto getObstacle() -> boost::optional<openscenario_msgs::msg::Obstacle> override;

  auto getVehicleCommand() -> const autoware_vehicle_msgs::msg::VehicleCommand override;

  auto getWaypoints() -> const openscenario_msgs::msg::WaypointsArray override;

  void onUpdate(double current_time, double step_time) override;

  auto ready() const -> bool override;

  void requestAcquirePosition(const openscenario_msgs::msg::LaneletPose &) override;

  void requestAssignRoute(const std::vector<openscenario_msgs::msg::LaneletPose> &) override;

  void requestLaneChange(const std::int64_t) override;

  auto setStatus(const openscenario_msgs::msg::EntityStatus & status) -> bool override;

  void setTargetSpeed(double, bool) override;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__EGO_ENTITY_HPP_
