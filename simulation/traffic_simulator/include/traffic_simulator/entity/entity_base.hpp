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

#ifndef TRAFFIC_SIMULATOR__ENTITY__ENTITY_BASE_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__ENTITY_BASE_HPP_

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <boost/optional.hpp>
#include <concealer/autoware.hpp>
#include <memory>
#include <queue>
#include <string>
#include <traffic_simulator/data_type/lane_change.hpp>
#include <traffic_simulator/data_type/speed_change.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/job/job_list.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_manager.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <traffic_simulator_msgs/msg/driver_model.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/entity_type.hpp>
#include <traffic_simulator_msgs/msg/obstacle.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <unordered_map>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace traffic_simulator
{
namespace entity
{
class EntityBase
{
public:
  explicit EntityBase(
    const std::string & name, const traffic_simulator_msgs::msg::EntityStatus & entity_status,
    const traffic_simulator_msgs::msg::EntitySubtype & subtype)
  : name(name),
    subtype(subtype),
    verbose(true),
    status_(entity_status),
    status_before_update_(status_),
    npc_logic_started_(false)
  {
  }

  virtual ~EntityBase() = default;

  virtual void appendDebugMarker(visualization_msgs::msg::MarkerArray &);

  virtual auto asAutoware() const -> concealer::Autoware &;

  virtual void cancelRequest();

  /*   */ auto get2DPolygon() const -> std::vector<geometry_msgs::msg::Point>;

  virtual auto getBoundingBox() const -> const traffic_simulator_msgs::msg::BoundingBox = 0;

  virtual auto getCurrentAction() const -> std::string = 0;

  /*   */ auto getDistanceToLaneBound() -> double;

  /*   */ auto getDistanceToLaneBound(std::int64_t lanelet_id) const -> double;

  /*   */ auto getDistanceToLaneBound(const std::vector<std::int64_t> &) const -> double;

  /*   */ auto getDistanceToLeftLaneBound() -> double;

  /*   */ auto getDistanceToLeftLaneBound(std::int64_t lanelet_id) const -> double;

  /*   */ auto getDistanceToLeftLaneBound(const std::vector<std::int64_t> &) const -> double;

  /*   */ auto getDistanceToRightLaneBound() -> double;

  /*   */ auto getDistanceToRightLaneBound(std::int64_t lanelet_id) const -> double;

  /*   */ auto getDistanceToRightLaneBound(const std::vector<std::int64_t> &) const -> double;

  virtual auto getDriverModel() const -> traffic_simulator_msgs::msg::DriverModel = 0;

  /*   */ auto getEntityStatusBeforeUpdate() const
    -> const traffic_simulator_msgs::msg::EntityStatus &;

  virtual auto getEntityType() const -> const traffic_simulator_msgs::msg::EntityType & = 0;

  virtual auto getEntityTypename() const -> const std::string & = 0;

  virtual auto getGoalPoses() -> std::vector<traffic_simulator_msgs::msg::LaneletPose> = 0;

  /*   */ auto getLinearJerk() const -> boost::optional<double>;

  /*   */ auto getLaneletPose() const -> boost::optional<traffic_simulator_msgs::msg::LaneletPose>;

  /*   */ auto getMapPose() const -> geometry_msgs::msg::Pose;

  /*   */ auto getMapPose(const geometry_msgs::msg::Pose &) -> geometry_msgs::msg::Pose;

  virtual auto getObstacle() -> boost::optional<traffic_simulator_msgs::msg::Obstacle> = 0;

  virtual auto getRouteLanelets(const double horizon = 100) -> std::vector<std::int64_t> = 0;

  /*   */ auto getStatus() const -> const traffic_simulator_msgs::msg::EntityStatus &;

  /*   */ auto getStandStillDuration() const -> double;

  /*   */ auto getVehicleParameters() const
    -> const boost::optional<traffic_simulator_msgs::msg::VehicleParameters>;

  virtual auto getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray = 0;

  /*   */ auto isNpcLogicStarted() const -> bool;

  virtual void onUpdate(double current_time, double step_time);

  virtual void requestAcquirePosition(const traffic_simulator_msgs::msg::LaneletPose &) = 0;

  virtual void requestAcquirePosition(const geometry_msgs::msg::Pose &) = 0;

  virtual void requestAssignRoute(
    const std::vector<traffic_simulator_msgs::msg::LaneletPose> &) = 0;

  virtual void requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> &) = 0;

  virtual void requestLaneChange(const std::int64_t){};

  virtual void requestLaneChange(const traffic_simulator::lane_change::Parameter &){};

  /*   */ void requestLaneChange(
    const lane_change::AbsoluteTarget &, const lane_change::TrajectoryShape,
    const lane_change::Constraint &);

  /*   */ void requestLaneChange(
    const lane_change::RelativeTarget &, const lane_change::TrajectoryShape,
    const lane_change::Constraint &);

  virtual void requestSpeedChange(
    const double target_speed, const speed_change::Transition, const speed_change::Constraint,
    const bool continuous);

  virtual void requestSpeedChange(
    const speed_change::RelativeTargetSpeed &, const speed_change::Transition,
    const speed_change::Constraint, const bool continuous);

  virtual void requestSpeedChange(double target_speed, bool continuous);

  virtual void requestSpeedChange(const speed_change::RelativeTargetSpeed &, bool continuous);

  virtual void requestWalkStraight();

  virtual void setAccelerationLimit(double acceleration) = 0;

  virtual void setDecelerationLimit(double deceleration) = 0;

  virtual void setDriverModel(const traffic_simulator_msgs::msg::DriverModel &) = 0;

  /*   */ void setEntityTypeList(
    const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityType> &);

  virtual void setHdMapUtils(const std::shared_ptr<hdmap_utils::HdMapUtils> &);

  /*   */ void setOtherStatus(
    const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityStatus> &);

  virtual auto setStatus(const traffic_simulator_msgs::msg::EntityStatus &) -> bool;

  virtual void setTrafficLightManager(
    const std::shared_ptr<traffic_simulator::TrafficLightManagerBase> &);

  virtual auto setVelocityLimit(double) -> void;

  virtual void startNpcLogic();

  /*   */ void stopAtEndOfRoad();

  /*   */ void updateEntityStatusTimestamp(const double current_time);

  /*   */ auto updateStandStillDuration(const double step_time) -> double;

  const std::string name;

  const traffic_simulator_msgs::msg::EntitySubtype subtype;

  bool verbose;

protected:
  traffic_simulator_msgs::msg::EntityStatus status_;

  traffic_simulator_msgs::msg::EntityStatus status_before_update_;

  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;
  std::shared_ptr<traffic_simulator::TrafficLightManagerBase> traffic_light_manager_;

  bool npc_logic_started_;

  std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityStatus> other_status_;
  std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityType> entity_type_list_;

  boost::optional<double> linear_jerk_;

  double stand_still_duration_ = 0.0;

  boost::optional<double> target_speed_;
  traffic_simulator::job::JobList job_list_;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__ENTITY_BASE_HPP_
