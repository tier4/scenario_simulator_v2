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
#include <concealer/autoware.hpp>
#include <memory>
#include <openscenario_msgs/msg/entity_type.hpp>
#include <string>
#include <traffic_simulator/entity/vehicle_entity.hpp>
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
  // NOTE: One day we will have to do simultaneous simulations of multiple Autowares.
  static std::unordered_map<std::string, concealer::Autoware> autowares;

  bool autoware_initialized = false;  // TODO (yamacir-kit) REMOVE THIS!!!

  const std::shared_ptr<SimModelInterface> vehicle_model_ptr_;

  boost::optional<geometry_msgs::msg::Pose> initial_pose_;

  boost::optional<double> previous_linear_velocity_, previous_angular_velocity_;

  const VehicleModelType vehicle_model_type_;

public:
  EgoEntity() = delete;

  /* ---- NOTE -----------------------------------------------------------------
   *
   *  This constructor builds an Ego-type entity with an ambiguous initial
   *  state. In this case, the values for status_ and current_kinematic_state_
   *  are boost::none, respectively.
   *
   *  This constructor is used for the purpose of delaying the transmission of
   *  the initial position from the entity's spawn. If you build an ego-type
   *  entity with this constructor, you must explicitly call setStatus at least
   *  once before the first onUpdate call to establish location and kinematic
   *  state.
   *
   *  For OpenSCENARIO, setStatus before the onUpdate call is called by
   *  TeleportAction in the Storyboard.Init section.
   *
   * ------------------------------------------------------------------------ */
  explicit EgoEntity(
    const std::string & name,                          //
    const boost::filesystem::path & lanelet2_map_osm,  //
    const double step_time,                            //
    const openscenario_msgs::msg::VehicleParameters & parameters);

  ~EgoEntity() override;

  EgoEntity(const EgoEntity &) = delete;

  EgoEntity & operator=(const EgoEntity &) = delete;

public:
  void engage() override;

  auto getCurrentAction() const -> const std::string override;

  auto getEntityStatus(const double, const double) const
    -> const openscenario_msgs::msg::EntityStatus;

  auto getEntityTypename() const -> const std::string & override;

  auto getObstacle() -> boost::optional<openscenario_msgs::msg::Obstacle> override;

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
