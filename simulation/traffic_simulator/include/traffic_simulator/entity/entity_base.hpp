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

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
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
#include <traffic_simulator/utils/distance.hpp>
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
using EuclideanDistancesMap = std::unordered_map<std::pair<std::string, std::string>, double>;
class EntityBase : public std::enable_shared_from_this<EntityBase>
{
public:
  explicit EntityBase(
    const std::string & name, const CanonicalizedEntityStatus & entity_status,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr);

  EntityBase(const EntityBase &) = delete;

  EntityBase & operator=(const EntityBase &) = delete;

  EntityBase(EntityBase &&) noexcept = delete;

  EntityBase & operator=(EntityBase &&) noexcept = delete;

  virtual ~EntityBase() = default;

  template <typename EntityType>
  /*   */ auto is() const -> bool
  {
    return dynamic_cast<EntityType const *>(this) != nullptr;
  }

  template <typename EntityType>
  /*   */ auto as() -> EntityType &
  {
    if (const auto derived_ptr = dynamic_cast<EntityType *>(this); !derived_ptr) {
      THROW_SEMANTIC_ERROR(
        "Entity ", std::quoted(name), " is not ", std::quoted(typeid(EntityType).name()), "type");
    } else {
      return *derived_ptr;
    }
  }

  template <typename EntityType>
  /*   */ auto as() const -> const EntityType &
  {
    if (const auto derived_ptr = dynamic_cast<EntityType const *>(this); !derived_ptr) {
      THROW_SEMANTIC_ERROR(
        "Entity ", std::quoted(name), " is not ", std::quoted(typeid(EntityType).name()), "type");
    } else {
      return *derived_ptr;
    }
  }

  virtual void appendDebugMarker(visualization_msgs::msg::MarkerArray & /*unused*/);

  virtual void cancelRequest();

  // clang-format off
#define DEFINE_GETTER(NAME, TYPE, RETURN_VARIABLE)            \
  /**                                                         \
   @brief Get NAME of the entity.                             \
   @return NAME of the entity.                                \
   @note This function was defined by DEFINE_GETTER function. \
   */                                                         \
  /*   */ auto get##NAME() const noexcept->TYPE { return RETURN_VARIABLE; }

  DEFINE_GETTER(BoundingBox,                     const traffic_simulator_msgs::msg::BoundingBox &,   status_->getBoundingBox())
  DEFINE_GETTER(CanonicalizedStatus,             const CanonicalizedEntityStatus &,                  *status_)
  DEFINE_GETTER(CanonicalizedStatusBeforeUpdate, const CanonicalizedEntityStatus &,                  status_before_update_)
  DEFINE_GETTER(CurrentAccel,                    const geometry_msgs::msg::Accel &,                  status_->getAccel())
  DEFINE_GETTER(CurrentTwist,                    const geometry_msgs::msg::Twist &,                  status_->getTwist())
  DEFINE_GETTER(DynamicConstraints,              traffic_simulator_msgs::msg::DynamicConstraints,    getBehaviorParameter().dynamic_constraints)
  DEFINE_GETTER(EntitySubtype,                   const traffic_simulator_msgs::msg::EntitySubtype &, status_->getSubtype())
  DEFINE_GETTER(EntityType,                      const traffic_simulator_msgs::msg::EntityType &,    status_->getType())
  DEFINE_GETTER(LinearJerk,                      double,                                             status_->getLinearJerk())
  DEFINE_GETTER(MapPose,                         const geometry_msgs::msg::Pose &,                   status_->getMapPose())
  DEFINE_GETTER(StandStillDuration,              double,                                             stand_still_duration_)
  DEFINE_GETTER(TraveledDistance,                double,                                             traveled_distance_)
  DEFINE_GETTER(Name,                            const std::string &,                                status_->getName())
  // clang-format on
#undef DEFINE_GETTER

  /*   */ auto get2DPolygon() const -> std::vector<geometry_msgs::msg::Point>;

  virtual auto getCurrentAction() const -> std::string = 0;

  virtual auto getBehaviorParameter() const -> traffic_simulator_msgs::msg::BehaviorParameter = 0;

  virtual auto getDefaultDynamicConstraints() const
    -> const traffic_simulator_msgs::msg::DynamicConstraints & = 0;

  virtual auto getEntityTypename() const -> const std::string & = 0;

  virtual auto getGoalPoses() -> std::vector<CanonicalizedLaneletPose> = 0;

  /*   */ auto isStopped() const -> bool;

  /*   */ auto isNearbyPosition(const geometry_msgs::msg::Pose & pose, const double tolerance) const
    -> bool;

  /*   */ auto isNearbyPosition(
    const CanonicalizedLaneletPose & lanelet_pose, const double tolerance) const -> bool;

  /*   */ auto isInLanelet() const -> bool { return status_->isInLanelet(); };

  /*   */ auto isInLanelet(
    const lanelet::Id lanelet_id, std::optional<double> tolerance = std::nullopt) const -> bool;

  /*   */ auto getCanonicalizedLaneletPose() const -> std::optional<CanonicalizedLaneletPose>;

  /*   */ auto getCanonicalizedLaneletPose(const double matching_distance) const
    -> std::optional<CanonicalizedLaneletPose>;

  virtual auto getMaxAcceleration() const -> double = 0;

  virtual auto getMaxDeceleration() const -> double = 0;

  virtual auto getDefaultMatchingDistanceForLaneletPoseCalculation() const -> double;

  virtual auto getObstacle() -> std::optional<traffic_simulator_msgs::msg::Obstacle> = 0;

  virtual auto getRouteLanelets(const double horizon = 100.0) -> lanelet::Ids = 0;

  virtual auto getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray = 0;

  virtual auto onUpdate(const double current_time, const double step_time) -> void;

  virtual auto onPostUpdate(const double current_time, const double step_time) -> void;

  /*   */ void resetDynamicConstraints();

  virtual void requestAcquirePosition(const CanonicalizedLaneletPose &) = 0;

  virtual void requestAcquirePosition(const geometry_msgs::msg::Pose &) = 0;

  virtual void requestAssignRoute(const std::vector<CanonicalizedLaneletPose> &) = 0;

  virtual void requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> &) = 0;

  virtual auto requestLaneChange(const lanelet::Id) -> void
  {
    /**
     * @note There are Entities such as MiscObjectEntity for which lane change is not possible,
     * and since it is necessary to implement appropriate overrides for each Entity, no operation is performed on the base type.
     */
  }

  virtual auto requestLaneChange(const lane_change::Parameter &) -> void
  {
    /**
     * @note There are Entities such as MiscObjectEntity for which lane change is not possible,
     * and since it is necessary to implement appropriate overrides for each Entity, no operation is performed on the base type.
     */
  }

  /*   */ auto requestLaneChange(const lane_change::Direction & direction) -> void;

  /*   */ auto requestLaneChange(
    const lane_change::AbsoluteTarget & target, const lane_change::TrajectoryShape trajectory_shape,
    const lane_change::Constraint & constraint) -> void;

  /*   */ auto requestLaneChange(
    const lane_change::RelativeTarget & target, const lane_change::TrajectoryShape trajectory_shape,
    const lane_change::Constraint & constraint) -> void;

  virtual auto requestSpeedChange(
    const double target_speed, const speed_change::Transition,
    const speed_change::Constraint constraint, const bool continuous) -> void;

  virtual void requestSpeedChange(
    const speed_change::RelativeTargetSpeed & target_speed,
    const speed_change::Transition transition, const speed_change::Constraint constraint,
    const bool continuous);

  virtual void requestSpeedChange(const double target_speed, const bool continuous);

  virtual void requestSpeedChange(
    const speed_change::RelativeTargetSpeed & target_speed, const bool continuous);

  virtual auto isControlledBySimulator() const -> bool;

  virtual auto setControlledBySimulator(const bool /*unused*/) -> void;

  virtual auto requestFollowTrajectory(
    const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & /*unused*/) -> void;

  virtual void requestWalkStraight();

  virtual void setAccelerationLimit(const double acceleration) = 0;

  virtual void setAccelerationRateLimit(const double acceleration_rate) = 0;

  virtual void setDecelerationLimit(const double deceleration) = 0;

  virtual void setDecelerationRateLimit(const double deceleration_rate) = 0;

  /*   */ void setDynamicConstraints(
    const traffic_simulator_msgs::msg::DynamicConstraints & constraints);

  virtual void setBehaviorParameter(const traffic_simulator_msgs::msg::BehaviorParameter &) = 0;

  /*   */ void setOtherStatus(
    const std::unordered_map<std::string, CanonicalizedEntityStatus> & status);

  /*   */ auto setCanonicalizedStatus(const CanonicalizedEntityStatus & status) -> void;

  virtual auto setStatus(const EntityStatus & status) -> void;

  virtual auto setStatus(const EntityStatus & status, const lanelet::Ids & lanelet_ids) -> void;

  virtual auto setStatus(
    const geometry_msgs::msg::Pose & map_pose,
    const traffic_simulator_msgs::msg::ActionStatus & action_status =
      helper::constructActionStatus()) -> void;

  virtual auto setStatus(
    const geometry_msgs::msg::Pose & reference_pose, const geometry_msgs::msg::Pose & relative_pose,
    const traffic_simulator_msgs::msg::ActionStatus & action_status =
      helper::constructActionStatus()) -> void;

  virtual auto setStatus(
    const geometry_msgs::msg::Pose & reference_pose,
    const geometry_msgs::msg::Point & relative_position,
    const geometry_msgs::msg::Vector3 & relative_rpy,
    const traffic_simulator_msgs::msg::ActionStatus & action_status =
      helper::constructActionStatus()) -> void;

  virtual auto setStatus(
    const CanonicalizedLaneletPose & canonicalized_lanelet_pose,
    const traffic_simulator_msgs::msg::ActionStatus & action_status =
      helper::constructActionStatus()) -> void;

  virtual auto setStatus(
    const LaneletPose & lanelet_pose,
    const traffic_simulator_msgs::msg::ActionStatus & action_status =
      helper::constructActionStatus()) -> void;

  virtual auto setLinearAcceleration(const double linear_acceleration) -> void;

  virtual auto setLinearVelocity(const double linear_velocity) -> void;

  virtual void setTrafficLights(
    const std::shared_ptr<traffic_simulator::TrafficLightsBase> & traffic_lights);

  virtual auto activateOutOfRangeJob(
    const double min_velocity, const double max_velocity, const double min_acceleration,
    const double max_acceleration, const double min_jerk, const double max_jerk) -> void;

  virtual auto setVelocityLimit(const double) -> void = 0;

  virtual auto setMapPose(const geometry_msgs::msg::Pose & map_pose) -> void;

  /*   */ auto setTwist(const geometry_msgs::msg::Twist & twist) -> void;

  /*   */ auto setAcceleration(const geometry_msgs::msg::Accel & accel) -> void;

  /*   */ auto setAction(const std::string & action) -> void;

  /*   */ auto setLinearJerk(const double liner_jerk) -> void;

  /*   */ void stopAtCurrentPosition();

  /*   */ void updateEntityStatusTimestamp(const double current_time);

  /*   */ auto requestSynchronize(
    const std::string & target_name, const CanonicalizedLaneletPose & target_sync_pose,
    const CanonicalizedLaneletPose & entity_target, const double target_speed,
    const double tolerance) -> bool;

  const std::string name;

  bool verbose;

  void setEuclideanDistancesMap(const std::shared_ptr<EuclideanDistancesMap> & distances);

protected:
  std::shared_ptr<CanonicalizedEntityStatus> status_;

  CanonicalizedEntityStatus status_before_update_;

  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;
  std::shared_ptr<traffic_simulator::TrafficLightsBase> traffic_lights_;

  double stand_still_duration_ = 0.0;
  double traveled_distance_ = 0.0;
  double prev_job_duration_ = 0.0;
  double step_time_ = 0.0;

  std::unordered_map<std::string, CanonicalizedEntityStatus> other_status_;

  std::optional<double> target_speed_;
  traffic_simulator::job::JobList job_list_;

  std::unique_ptr<traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner>
    speed_planner_;

  std::shared_ptr<EuclideanDistancesMap> euclidean_distances_map_;

private:
  virtual auto requestSpeedChangeWithConstantAcceleration(
    const double target_speed, const speed_change::Transition, const double acceleration,
    const bool continuous) -> void;
  virtual auto requestSpeedChangeWithConstantAcceleration(
    const speed_change::RelativeTargetSpeed & target_speed,
    const speed_change::Transition transition, const double acceleration, const bool continuous)
    -> void;
  virtual auto requestSpeedChangeWithTimeConstraint(
    const double target_speed, const speed_change::Transition, const double acceleration_time)
    -> void;
  virtual auto requestSpeedChangeWithTimeConstraint(
    const speed_change::RelativeTargetSpeed & target_speed,
    const speed_change::Transition transition, const double time) -> void;
  /*   */ auto isTargetSpeedReached(const double target_speed) const -> bool;
  /*   */ auto isTargetSpeedReached(const speed_change::RelativeTargetSpeed & target_speed) const
    -> bool;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__ENTITY_BASE_HPP_
