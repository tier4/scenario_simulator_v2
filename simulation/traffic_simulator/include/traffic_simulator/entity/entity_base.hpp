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
#include <concealer/field_operator_application.hpp>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <traffic_simulator/behavior/follow_trajectory.hpp>
#include <traffic_simulator/behavior/longitudinal_speed_planning.hpp>
#include <traffic_simulator/data_type/lane_change.hpp>
#include <traffic_simulator/data_type/speed_change.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/job/job_list.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_manager.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
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
  explicit EntityBase(const std::string & name, const traffic_simulator_msgs::msg::EntityStatus &);

  virtual ~EntityBase() = default;

  virtual void appendDebugMarker(visualization_msgs::msg::MarkerArray &);

  virtual auto asFieldOperatorApplication() const -> concealer::FieldOperatorApplication &;

  virtual void cancelRequest();

  /*   */ auto get2DPolygon() const -> std::vector<geometry_msgs::msg::Point>;

  virtual auto getCurrentAction() const -> std::string = 0;

  /*   */ auto getCurrentAccel() const -> geometry_msgs::msg::Accel;

  /*   */ auto getCurrentTwist() const -> geometry_msgs::msg::Twist;

  /*   */ auto getDistanceToLaneBound() -> double;

  /*   */ auto getDistanceToLaneBound(std::int64_t lanelet_id) const -> double;

  /*   */ auto getDistanceToLaneBound(const std::vector<std::int64_t> &) const -> double;

  /*   */ auto getDistanceToLeftLaneBound() -> double;

  /*   */ auto getDistanceToLeftLaneBound(std::int64_t lanelet_id) const -> double;

  /*   */ auto getDistanceToLeftLaneBound(const std::vector<std::int64_t> &) const -> double;

  /*   */ auto getDistanceToRightLaneBound() -> double;

  /*   */ auto getDistanceToRightLaneBound(std::int64_t lanelet_id) const -> double;

  /*   */ auto getDistanceToRightLaneBound(const std::vector<std::int64_t> &) const -> double;

  virtual auto getBehaviorParameter() const -> traffic_simulator_msgs::msg::BehaviorParameter = 0;

  virtual auto getDynamicConstraints() const
    -> const traffic_simulator_msgs::msg::DynamicConstraints;

  virtual auto getDefaultDynamicConstraints() const
    -> const traffic_simulator_msgs::msg::DynamicConstraints & = 0;

  /*   */ auto getEntityStatusBeforeUpdate() const
    -> const traffic_simulator_msgs::msg::EntityStatus &;

  virtual auto getEntityTypename() const -> const std::string & = 0;

  /*   */ auto getTraveledDistance() const -> double;

  virtual auto getGoalPoses() -> std::vector<traffic_simulator_msgs::msg::LaneletPose> = 0;

  /*   */ auto getLinearJerk() const -> double;

  /*   */ auto getLaneletPose() const -> std::optional<traffic_simulator_msgs::msg::LaneletPose>;

  /*   */ auto getLaneletPose(double matching_distance) const
    -> std::optional<traffic_simulator_msgs::msg::LaneletPose>;

  /*   */ auto getMapPose() const -> geometry_msgs::msg::Pose;

  /*   */ auto getMapPose(const geometry_msgs::msg::Pose &) -> geometry_msgs::msg::Pose;

  virtual auto getObstacle() -> std::optional<traffic_simulator_msgs::msg::Obstacle> = 0;

  virtual auto getRouteLanelets(double horizon = 100) const -> std::vector<std::int64_t> = 0;

  virtual auto fillLaneletPose(traffic_simulator_msgs::msg::EntityStatus & status) const
    -> void = 0;

  /*   */ auto getStatus() const -> const traffic_simulator_msgs::msg::EntityStatus &;

  /*   */ auto getStandStillDuration() const -> double;

  virtual auto getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray = 0;

  /*   */ auto isNpcLogicStarted() const -> bool;

  virtual void onUpdate(double current_time, double step_time);

  virtual void onPostUpdate(double current_time, double step_time);

  /*   */ void resetDynamicConstraints();

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
    const double, const speed_change::Transition, const speed_change::Constraint, const bool);

  virtual void requestSpeedChange(
    const speed_change::RelativeTargetSpeed &, const speed_change::Transition,
    const speed_change::Constraint, const bool);

  virtual void requestSpeedChange(double, bool);

  virtual void requestSpeedChange(const speed_change::RelativeTargetSpeed &, bool);

  virtual auto requestFollowTrajectory(
    const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> &) -> void;

  virtual void requestWalkStraight();

  virtual void setAccelerationLimit(double acceleration) = 0;

  virtual void setAccelerationRateLimit(double acceleration_rate) = 0;

  virtual void setDecelerationLimit(double deceleration) = 0;

  virtual void setDecelerationRateLimit(double deceleration_rate) = 0;

  /*   */ void setDynamicConstraints(const traffic_simulator_msgs::msg::DynamicConstraints &);

  virtual void setBehaviorParameter(const traffic_simulator_msgs::msg::BehaviorParameter &) = 0;

  /*   */ void setEntityTypeList(
    const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityType> &);

  virtual void setHdMapUtils(const std::shared_ptr<hdmap_utils::HdMapUtils> &);

  /*   */ void setOtherStatus(
    const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityStatus> &);

  virtual auto setStatus(const traffic_simulator_msgs::msg::EntityStatus &) -> void;

  virtual auto setLinearAcceleration(const double linear_acceleration) -> void;

  virtual auto setLinearVelocity(const double linear_velocity) -> void;

  virtual void setTrafficLightManager(
    const std::shared_ptr<traffic_simulator::TrafficLightManager> &);

  virtual auto activateOutOfRangeJob(
    double min_velocity, double max_velocity, double min_acceleration, double max_acceleration,
    double min_jerk, double max_jerk) -> void;

  virtual auto setVelocityLimit(double) -> void;

  virtual void startNpcLogic();

  /*   */ void stopAtEndOfRoad();

  /*   */ void updateEntityStatusTimestamp(const double current_time);

  /*   */ auto updateStandStillDuration(const double step_time) -> double;

  /*   */ auto updateTraveledDistance(const double step_time) -> double;

  virtual auto fillLaneletPose(
    traffic_simulator_msgs::msg::EntityStatus & status, bool include_crosswalk) const -> void final;

  const std::string name;

  bool verbose;

protected:
  traffic_simulator_msgs::msg::EntityStatus status_;

  traffic_simulator_msgs::msg::EntityStatus status_before_update_;

  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;
  std::shared_ptr<traffic_simulator::TrafficLightManager> traffic_light_manager_;

  bool npc_logic_started_ = false;
  double stand_still_duration_ = 0.0;
  double traveled_distance_ = 0.0;

  std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityStatus> other_status_;
  std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityType> entity_type_list_;

  std::optional<double> target_speed_;
  traffic_simulator::job::JobList job_list_;

  std::unique_ptr<traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner>
    speed_planner_;

private:
  virtual auto requestSpeedChangeWithConstantAcceleration(
    const double target_speed, const speed_change::Transition, double acceleration,
    const bool continuous) -> void;
  virtual auto requestSpeedChangeWithConstantAcceleration(
    const speed_change::RelativeTargetSpeed & target_speed,
    const speed_change::Transition transition, double acceleration, const bool continuous) -> void;
  virtual auto requestSpeedChangeWithTimeConstraint(
    const double target_speed, const speed_change::Transition, double acceleration_time) -> void;
  virtual auto requestSpeedChangeWithTimeConstraint(
    const speed_change::RelativeTargetSpeed & target_speed,
    const speed_change::Transition transition, double time) -> void;
  /*   */ auto isTargetSpeedReached(double target_speed) const -> bool;
  /*   */ auto isTargetSpeedReached(const speed_change::RelativeTargetSpeed & target_speed) const
    -> bool;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__ENTITY_BASE_HPP_
