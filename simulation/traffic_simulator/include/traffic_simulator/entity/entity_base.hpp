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
#include <concealer/autoware.hpp>
#include <memory>
#include <optional>
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
  const std::string type;

  const std::string name;

  EntityBase(const std::string & name, const traffic_simulator_msgs::msg::EntitySubtype & subtype);

  virtual ~EntityBase() = default;

public:
  virtual void appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array);

  virtual auto asAutoware() const -> concealer::Autoware &;

  virtual auto getBoundingBox() const -> const traffic_simulator_msgs::msg::BoundingBox = 0;

  virtual auto getCurrentAction() const -> const std::string = 0;

  /*   */ auto getEntityStatusBeforeUpdate() const
    -> const std::optional<traffic_simulator_msgs::msg::EntityStatus>
  {
    return status_before_update_;
  }

  /*   */ auto getEntityType() const -> const auto & { return entity_type_; }

  virtual auto getEntityTypename() const -> const std::string & = 0;

  /*   */ auto getLinearJerk() const { return linear_jerk_; }

  /*   */ auto getLaneletPose() const -> std::optional<traffic_simulator_msgs::msg::LaneletPose>;

  virtual auto getObstacle() -> std::optional<traffic_simulator_msgs::msg::Obstacle> = 0;

  virtual auto getRouteLanelets(const double horizon = 100) -> std::vector<std::int64_t> = 0;

  /*   */ auto getStatus() const -> const traffic_simulator_msgs::msg::EntityStatus;

  /*   */ auto getStandStillDuration() const -> std::optional<double>;

  /*   */ auto getVisibility() { return visibility_; }

  /*   */ auto getVehicleParameters() const
    -> const std::optional<traffic_simulator_msgs::msg::VehicleParameters>
  {
    return std::nullopt;
  }

  virtual auto get2DPolygon() const -> std::vector<geometry_msgs::msg::Point>;

  /*   */ auto getDistanceToLaneBound() -> double;

  /*   */ auto getDistanceToLaneBound(std::int64_t lanelet_id) const -> double;

  /*   */ auto getDistanceToLaneBound(const std::vector<std::int64_t> & lanelet_ids) const
    -> double;

  /*   */ auto getDistanceToLeftLaneBound() -> double;

  /*   */ auto getDistanceToLeftLaneBound(std::int64_t lanelet_id) const -> double;

  /*   */ auto getDistanceToLeftLaneBound(const std::vector<std::int64_t> & lanelet_ids) const
    -> double;

  /*   */ auto getDistanceToRightLaneBound() -> double;

  /*   */ auto getDistanceToRightLaneBound(std::int64_t lanelet_id) const -> double;

  /*   */ auto getDistanceToRightLaneBound(const std::vector<std::int64_t> & lanelet_ids) const
    -> double;

  /*   */ auto getMapPose() const -> geometry_msgs::msg::Pose;

  /*   */ auto getMapPose(const geometry_msgs::msg::Pose & relative_pose)
    -> geometry_msgs::msg::Pose;

  virtual auto getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray = 0;

  virtual auto getGoalPoses() -> std::vector<traffic_simulator_msgs::msg::LaneletPose> = 0;

  virtual auto getDriverModel() const -> traffic_simulator_msgs::msg::DriverModel = 0;

  virtual void setDriverModel(const traffic_simulator_msgs::msg::DriverModel &) = 0;

  virtual void setAccelerationLimit(double acceleration);

  virtual void setDecelerationLimit(double deceleration);

  /*   */ void setEntityTypeList(
    const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityType> &
      entity_type_list)
  {
    entity_type_list_ = entity_type_list;
  }

  virtual void setHdMapUtils(const std::shared_ptr<hdmap_utils::HdMapUtils> & ptr)
  {
    hdmap_utils_ptr_ = ptr;
  }

  /*   */ void setOtherStatus(
    const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityStatus> & status);

  virtual auto setStatus(const traffic_simulator_msgs::msg::EntityStatus & status) -> bool;

  virtual void setTrafficLightManager(
    const std::shared_ptr<traffic_simulator::TrafficLightManagerBase> & ptr)
  {
    traffic_light_manager_ = ptr;
  }

  virtual auto setVelocityLimit(double) -> void {}

  /*   */ void setVerbose(bool verbose) { verbose_ = verbose; }

  /*   */ auto setVisibility(const bool visibility) { return visibility_ = visibility; }

  virtual void onUpdate(double current_time, double step_time);

  virtual void requestAcquirePosition(
    const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose) = 0;

  virtual void requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose) = 0;

  virtual void requestAssignRoute(
    const std::vector<traffic_simulator_msgs::msg::LaneletPose> & waypoints) = 0;

  virtual void requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> & waypoints) = 0;

  virtual void requestSpeedChange(
    const double target_speed, const speed_change::Transition transition,
    const speed_change::Constraint constraint, const bool continuous);

  virtual void requestSpeedChange(
    const speed_change::RelativeTargetSpeed & target_speed,
    const speed_change::Transition transition, const speed_change::Constraint constraint,
    const bool continuous);

  virtual void requestSpeedChange(double target_speed, bool continuous);

  virtual void requestSpeedChange(
    const speed_change::RelativeTargetSpeed & target_speed, bool continuous);

  virtual void requestLaneChange(const std::int64_t){};

  virtual void requestLaneChange(const traffic_simulator::lane_change::Parameter &){};

  void requestLaneChange(
    const traffic_simulator::lane_change::AbsoluteTarget & target,
    const traffic_simulator::lane_change::TrajectoryShape trajectory_shape,
    const traffic_simulator::lane_change::Constraint & constraint);

  void requestLaneChange(
    const traffic_simulator::lane_change::RelativeTarget & target,
    const traffic_simulator::lane_change::TrajectoryShape trajectory_shape,
    const traffic_simulator::lane_change::Constraint & constraint);

  virtual void requestWalkStraight()
  {
    THROW_SEMANTIC_ERROR(getEntityTypename(), " type entities do not support WalkStraightAction");
  }

  /*   */ auto statusSet() const noexcept { return static_cast<bool>(status_); }

  /*   */ void stopAtEndOfRoad();

  /*   */ void updateEntityStatusTimestamp(const double current_time);

  /*   */ void updateStandStillDuration(const double step_time);

  virtual void cancelRequest()
  {
    THROW_SEMANTIC_ERROR(getEntityTypename(), " type entities do not support cancel request");
  }

  /*   */ bool isNpcLogicStarted() const;

  virtual void startNpcLogic();

protected:
  std::optional<traffic_simulator_msgs::msg::LaneletPose> next_waypoint_;
  std::optional<traffic_simulator_msgs::msg::EntityStatus> status_;
  std::optional<traffic_simulator_msgs::msg::EntityStatus> status_before_update_;

  std::queue<traffic_simulator_msgs::msg::LaneletPose> waypoints_;

  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;
  std::shared_ptr<traffic_simulator::TrafficLightManagerBase> traffic_light_manager_;
  std::shared_ptr<math::geometry::CatmullRomSpline> spline_;

  bool verbose_;
  bool visibility_;
  bool npc_logic_started_;

  std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityStatus> other_status_;
  std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityType> entity_type_list_;

  std::optional<double> linear_jerk_;
  std::optional<double> stand_still_duration_;

  visualization_msgs::msg::MarkerArray current_marker_;
  traffic_simulator_msgs::msg::EntityType entity_type_;
  const traffic_simulator_msgs::msg::EntitySubtype entity_subtype_;

  std::optional<double> target_speed_;
  traffic_simulator::job::JobList job_list_;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__ENTITY_BASE_HPP_
