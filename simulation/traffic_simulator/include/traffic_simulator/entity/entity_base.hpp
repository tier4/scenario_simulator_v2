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
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator/data_type/lane_change.hpp>
#include <traffic_simulator/data_type/speed_change.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/job/job_list.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights.hpp>
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
  explicit EntityBase(
    const std::string & name, const CanonicalizedEntityStatus &,
    const std::shared_ptr<hdmap_utils::HdMapUtils> &);

  virtual ~EntityBase() = default;

  template <typename EntityType>
  /*   */ auto is() const -> bool
  {
    return dynamic_cast<EntityType const *>(this) != nullptr;
  }

  virtual void appendDebugMarker(visualization_msgs::msg::MarkerArray &);

  virtual void cancelRequest();

  // clang-format off
#define DEFINE_GETTER(NAME, TYPE, RETURN_VARIABLE)            \
  /**                                                         \
   @brief Get NAME of the entity.                             \
   @return NAME of the entity.                                \
   @note This function was defined by DEFINE_GETTER function. \
   */                                                         \
  /*   */ auto get##NAME() const noexcept->TYPE { return RETURN_VARIABLE; }

  DEFINE_GETTER(BoundingBox,              traffic_simulator_msgs::msg::BoundingBox,        static_cast<EntityStatus>(getStatus()).bounding_box)
  DEFINE_GETTER(CurrentAccel,             geometry_msgs::msg::Accel,                       static_cast<EntityStatus>(getStatus()).action_status.accel)
  DEFINE_GETTER(CurrentTwist,             geometry_msgs::msg::Twist,                       static_cast<EntityStatus>(getStatus()).action_status.twist)
  DEFINE_GETTER(DynamicConstraints,       traffic_simulator_msgs::msg::DynamicConstraints, getBehaviorParameter().dynamic_constraints)
  DEFINE_GETTER(EntityStatusBeforeUpdate, const CanonicalizedEntityStatus &,               status_before_update_)
  DEFINE_GETTER(EntitySubtype,            traffic_simulator_msgs::msg::EntitySubtype,      static_cast<EntityStatus>(getStatus()).subtype)
  DEFINE_GETTER(LinearJerk,               double,                                          static_cast<EntityStatus>(getStatus()).action_status.linear_jerk)
  DEFINE_GETTER(MapPose,                  geometry_msgs::msg::Pose,                        static_cast<EntityStatus>(getStatus()).pose)
  DEFINE_GETTER(StandStillDuration,       double,                                          stand_still_duration_)
  DEFINE_GETTER(Status,                   const CanonicalizedEntityStatus &,               status_)
  DEFINE_GETTER(TraveledDistance,         double,                                          traveled_distance_)
  DEFINE_GETTER(Name,                     const std::string &,                             getStatus().getName())

  // clang-format on
#undef DEFINE_GETTER

  /*   */ auto get2DPolygon() const -> std::vector<geometry_msgs::msg::Point>;

  virtual auto getCurrentAction() const -> std::string = 0;

  virtual auto getBehaviorParameter() const -> traffic_simulator_msgs::msg::BehaviorParameter = 0;

  virtual auto getDefaultDynamicConstraints() const
    -> const traffic_simulator_msgs::msg::DynamicConstraints & = 0;

  virtual auto getEntityType() const -> const traffic_simulator_msgs::msg::EntityType & = 0;

  virtual auto getEntityTypename() const -> const std::string & = 0;

  virtual auto getGoalPoses() -> std::vector<CanonicalizedLaneletPose> = 0;

  /*   */ auto isStopping() const -> bool;

  /*   */ auto isInPosition(
    const geometry_msgs::msg::Pose & target_pose, const double tolerance) const -> bool;

  /*   */ auto isInPosition(
    const CanonicalizedLaneletPose & lanelet_pose, const double tolerance) const -> bool;

  /*   */ auto isInLanelet() const -> bool { return status_.isInLanelet(); };

  /*   */ auto isInLanelet(
    const lanelet::Id lanelet_id, std::optional<double> tolerance = std::nullopt) const -> bool;

  /*   */ auto getCanonicalizedLaneletPose() const -> std::optional<CanonicalizedLaneletPose>;

  /*   */ auto getCanonicalizedLaneletPose(double matching_distance) const
    -> std::optional<CanonicalizedLaneletPose>;

  virtual auto getDefaultMatchingDistanceForLaneletPoseCalculation() const -> double;

  virtual auto getObstacle() -> std::optional<traffic_simulator_msgs::msg::Obstacle> = 0;

  virtual auto getRouteLanelets(double horizon = 100) -> lanelet::Ids = 0;

  virtual auto getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray = 0;

  virtual auto onUpdate(const double current_time, const double step_time) -> void;

  virtual auto onPostUpdate(const double current_time, const double step_time) -> void;

  /*   */ void resetDynamicConstraints();

  virtual void requestAcquirePosition(const CanonicalizedLaneletPose &) = 0;

  virtual void requestAcquirePosition(const geometry_msgs::msg::Pose &) = 0;

  virtual void requestAssignRoute(const std::vector<CanonicalizedLaneletPose> &) = 0;

  virtual void requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> &) = 0;

  virtual void requestLaneChange(const lanelet::Id){};

  virtual void requestLaneChange(const lane_change::Parameter &){};

  /*   */ auto requestLaneChange(const lane_change::Direction & direction) -> void;

  /*   */ auto requestLaneChange(
    const lane_change::AbsoluteTarget & target, const lane_change::TrajectoryShape trajectory_shape,
    const lane_change::Constraint & constraint) -> void;

  /*   */ auto requestLaneChange(
    const lane_change::RelativeTarget & target, const lane_change::TrajectoryShape trajectory_shape,
    const lane_change::Constraint & constraint) -> void;

  virtual auto requestSpeedChange(
    const double, const speed_change::Transition, const speed_change::Constraint, const bool)
    -> void;

  virtual void requestSpeedChange(
    const speed_change::RelativeTargetSpeed &, const speed_change::Transition,
    const speed_change::Constraint, const bool);

  virtual void requestSpeedChange(double, bool);

  virtual void requestSpeedChange(const speed_change::RelativeTargetSpeed &, bool);

  virtual void requestClearRoute();

  virtual auto isControlledBySimulator() const -> bool;

  virtual auto setControlledBySimulator(bool) -> void;

  virtual auto requestFollowTrajectory(
    const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> &) -> void;

  virtual void requestWalkStraight();

  virtual void setAccelerationLimit(double acceleration) = 0;

  virtual void setAccelerationRateLimit(double acceleration_rate) = 0;

  virtual void setDecelerationLimit(double deceleration) = 0;

  virtual void setDecelerationRateLimit(double deceleration_rate) = 0;

  /*   */ void setDynamicConstraints(const traffic_simulator_msgs::msg::DynamicConstraints &);

  virtual void setBehaviorParameter(const traffic_simulator_msgs::msg::BehaviorParameter &) = 0;

  /*   */ void setOtherStatus(const std::unordered_map<std::string, CanonicalizedEntityStatus> &);

  virtual void setStatus(const EntityStatus & status);

  virtual auto setCanonicalizedStatus(const CanonicalizedEntityStatus &) -> void;

  virtual auto setLinearAcceleration(const double linear_acceleration) -> void;

  virtual auto setLinearVelocity(const double linear_velocity) -> void;

  virtual void setTrafficLights(const std::shared_ptr<traffic_simulator::TrafficLightsBase> &);

  virtual auto activateOutOfRangeJob(
    double min_velocity, double max_velocity, double min_acceleration, double max_acceleration,
    double min_jerk, double max_jerk) -> void;

  virtual auto setVelocityLimit(double) -> void = 0;

  virtual auto setMapPose(const geometry_msgs::msg::Pose & map_pose) -> void;

  /*   */ auto setTwist(const geometry_msgs::msg::Twist & twist) -> void;

  /*   */ auto setAcceleration(const geometry_msgs::msg::Accel & accel) -> void;

  /*   */ auto setAction(const std::string & action) -> void;

  /*   */ auto setLinearJerk(const double liner_jerk) -> void;

  /*   */ void stopAtCurrentPosition();

  /*   */ void updateEntityStatusTimestamp(const double current_time);

  /*   */ auto updateStandStillDuration(const double step_time) -> double;

  /*   */ auto updateTraveledDistance(const double step_time) -> double;

  const std::string name;

  bool verbose;

protected:
  CanonicalizedEntityStatus status_;

  CanonicalizedEntityStatus status_before_update_;

  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;
  std::shared_ptr<traffic_simulator::TrafficLightsBase> traffic_lights_;

  double stand_still_duration_ = 0.0;
  double traveled_distance_ = 0.0;

  std::unordered_map<std::string, CanonicalizedEntityStatus> other_status_;

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
