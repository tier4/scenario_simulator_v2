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

#include <geometry/bounding_box.hpp>
#include <geometry/distance.hpp>
#include <geometry/transform.hpp>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <traffic_simulator/entity/entity_base.hpp>
#include <unordered_map>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
EntityBase::EntityBase(
  const std::string & name, const CanonicalizedEntityStatus & entity_status,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
: name(name),
  verbose(true),
  status_(entity_status),
  status_before_update_(status_),
  hdmap_utils_ptr_(hdmap_utils_ptr),
  npc_logic_started_(false)
{
  if (name != static_cast<EntityStatus>(entity_status).name) {
    THROW_SIMULATION_ERROR(
      "The name of the entity does not match the name of the entity listed in entity_status.",
      " The name of the entity is ", name,
      " and the name of the entity listed in entity_status is ",
      static_cast<EntityStatus>(entity_status).name);
  }
}

void EntityBase::appendDebugMarker(visualization_msgs::msg::MarkerArray &) {}

auto EntityBase::asFieldOperatorApplication() const -> concealer::FieldOperatorApplication &
{
  throw common::Error(
    "An operation was requested for Entity ", std::quoted(name),
    " that is valid only for the entity controlled by Autoware, but ", std::quoted(name),
    " is not the entity controlled by Autoware.");
}

void EntityBase::cancelRequest() {}

auto EntityBase::get2DPolygon() const -> std::vector<geometry_msgs::msg::Point>
{
  return math::geometry::toPolygon2D(getBoundingBox());
}

auto EntityBase::getLaneletPose() const -> std::optional<CanonicalizedLaneletPose>
{
  if (laneMatchingSucceed()) {
    return CanonicalizedLaneletPose(status_.getLaneletPose(), hdmap_utils_ptr_);
  }
  return std::nullopt;
}

auto EntityBase::getLaneletPose(double matching_distance) const
  -> std::optional<CanonicalizedLaneletPose>
{
  if (traffic_simulator_msgs::msg::EntityType::PEDESTRIAN == getEntityType().type) {
    if (
      const auto lanelet_pose =
        hdmap_utils_ptr_->toLaneletPose(getMapPose(), getBoundingBox(), true, matching_distance)) {
      return CanonicalizedLaneletPose(lanelet_pose.value(), hdmap_utils_ptr_);
    }
  } else {
    if (
      const auto lanelet_pose =
        hdmap_utils_ptr_->toLaneletPose(getMapPose(), getBoundingBox(), false, matching_distance)) {
      return CanonicalizedLaneletPose(lanelet_pose.value(), hdmap_utils_ptr_);
    }
  }
  return std::nullopt;
}

auto EntityBase::fillLaneletPose(CanonicalizedEntityStatus & status, bool include_crosswalk) -> void
{
  const auto unique_route_lanelets = traffic_simulator::helper::getUniqueValues(getRouteLanelets());

  std::optional<traffic_simulator_msgs::msg::LaneletPose> lanelet_pose;
  auto status_non_canonicalized = static_cast<EntityStatus>(status);

  if (unique_route_lanelets.empty()) {
    lanelet_pose = hdmap_utils_ptr_->toLaneletPose(
      status_non_canonicalized.pose, getBoundingBox(), include_crosswalk,
      getDefaultMatchingDistanceForLaneletPoseCalculation());
  } else {
    lanelet_pose = hdmap_utils_ptr_->toLaneletPose(
      status_non_canonicalized.pose, unique_route_lanelets,
      getDefaultMatchingDistanceForLaneletPoseCalculation());
    if (!lanelet_pose) {
      lanelet_pose = hdmap_utils_ptr_->toLaneletPose(
        status_non_canonicalized.pose, getBoundingBox(), include_crosswalk,
        getDefaultMatchingDistanceForLaneletPoseCalculation());
    }
  }
  if (lanelet_pose) {
    math::geometry::CatmullRomSpline spline(
      hdmap_utils_ptr_->getCenterPoints(lanelet_pose->lanelet_id));
    if (const auto s_value = spline.getSValue(status_non_canonicalized.pose)) {
      status_non_canonicalized.pose.position.z = spline.getPoint(s_value.value()).z;
    }
  }

  status_non_canonicalized.lanelet_pose_valid = static_cast<bool>(lanelet_pose);
  if (status_non_canonicalized.lanelet_pose_valid) {
    status_non_canonicalized.lanelet_pose = lanelet_pose.value();
  }
  status = CanonicalizedEntityStatus(status_non_canonicalized, hdmap_utils_ptr_);
}

auto EntityBase::getMapPoseFromRelativePose(const geometry_msgs::msg::Pose & relative_pose) const
  -> geometry_msgs::msg::Pose
{
  tf2::Transform ref_transform, relative_transform;
  tf2::fromMsg(getMapPose(), ref_transform);
  tf2::fromMsg(relative_pose, relative_transform);
  geometry_msgs::msg::Pose ret;
  tf2::toMsg(ref_transform * relative_transform, ret);
  return ret;
}

auto EntityBase::getDefaultMatchingDistanceForLaneletPoseCalculation() const -> double
{
  return getBoundingBox().dimensions.y * 0.5 + 1.0;
}

auto EntityBase::isTargetSpeedReached(double target_speed) const -> bool
{
  return speed_planner_->isTargetSpeedReached(target_speed, getCurrentTwist());
}

auto EntityBase::isTargetSpeedReached(const speed_change::RelativeTargetSpeed & target_speed) const
  -> bool
{
  return isTargetSpeedReached(target_speed.getAbsoluteValue(getStatus(), other_status_));
}

void EntityBase::onUpdate(double /*current_time*/, double step_time)
{
  job_list_.update(step_time, job::Event::PRE_UPDATE);
  status_before_update_ = status_;
  speed_planner_ =
    std::make_unique<traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner>(
      step_time, name);
}

void EntityBase::onPostUpdate(double /*current_time*/, double step_time)
{
  job_list_.update(step_time, job::Event::POST_UPDATE);
}

void EntityBase::resetDynamicConstraints()
{
  setDynamicConstraints(getDefaultDynamicConstraints());
}

void EntityBase::requestLaneChange(
  const traffic_simulator::lane_change::AbsoluteTarget & target,
  const traffic_simulator::lane_change::TrajectoryShape trajectory_shape,
  const traffic_simulator::lane_change::Constraint & constraint)
{
  requestLaneChange(lane_change::Parameter(target, trajectory_shape, constraint));
}

void EntityBase::requestLaneChange(
  const traffic_simulator::lane_change::RelativeTarget & target,
  const traffic_simulator::lane_change::TrajectoryShape trajectory_shape,
  const traffic_simulator::lane_change::Constraint & constraint)
{
  lanelet::Id reference_lanelet_id = 0;
  const auto lanelet_pose = getLaneletPose();
  if (lanelet_pose && target.entity_name == name) {
    if (!lanelet_pose) {
      THROW_SEMANTIC_ERROR(
        "Target entity does not assigned to lanelet. Please check Target entity name : ",
        target.entity_name, " exists on lane.");
    }
    reference_lanelet_id = static_cast<LaneletPose>(lanelet_pose.value()).lanelet_id;
  } else {
    if (other_status_.find(target.entity_name) == other_status_.end()) {
      THROW_SEMANTIC_ERROR(
        "Target entity : ", target.entity_name, " does not exist. Please check ",
        target.entity_name, " exists.");
    }
    if (!other_status_.at(target.entity_name).laneMatchingSucceed()) {
      THROW_SEMANTIC_ERROR(
        "Target entity does not assigned to lanelet. Please check Target entity name : ",
        target.entity_name, " exists on lane.");
    }
    reference_lanelet_id =
      static_cast<EntityStatus>(other_status_.at(target.entity_name)).lanelet_pose.lanelet_id;
  }
  const auto lane_change_target_id = hdmap_utils_ptr_->getLaneChangeableLaneletId(
    reference_lanelet_id, target.direction, target.shift);
  if (lane_change_target_id) {
    requestLaneChange(
      traffic_simulator::lane_change::AbsoluteTarget(lane_change_target_id.value(), target.offset),
      trajectory_shape, constraint);
  } else {
    THROW_SEMANTIC_ERROR(
      "Failed to calculate absolute target lane. Please check the target lane exists.");
  }
}

void EntityBase::requestSpeedChangeWithConstantAcceleration(
  const double target_speed, const speed_change::Transition transition, double acceleration,
  const bool continuous)
{
  if (!continuous && isTargetSpeedReached(target_speed)) {
    return;
  }
  switch (transition) {
    case speed_change::Transition::LINEAR: {
      setLinearAcceleration(acceleration);
      requestSpeedChangeWithConstantAcceleration(
        target_speed, speed_change::Transition::AUTO, acceleration, continuous);
      break;
    }
    case speed_change::Transition::AUTO: {
      if (speed_planner_->isAccelerating(target_speed, getCurrentTwist())) {
        setAccelerationLimit(std::abs(acceleration));
        job_list_.append(
          /**
           * @brief Checking if the entity reaches target speed.
           */
          [this, target_speed](double) { return getCurrentTwist().linear.x >= target_speed; },
          /**
           * @brief Resets acceleration limit.
           */
          [this]() { resetDynamicConstraints(); }, job::Type::LINEAR_ACCELERATION, true,
          job::Event::POST_UPDATE);
      } else if (speed_planner_->isDecelerating(target_speed, getCurrentTwist())) {
        setDecelerationLimit(std::abs(acceleration));
        job_list_.append(
          /**
           * @brief Checking if the entity reaches target speed.
           */
          [this, target_speed](double) { return getCurrentTwist().linear.x <= target_speed; },
          /**
           * @brief Resets deceleration limit.
           */
          [this]() { resetDynamicConstraints(); }, job::Type::LINEAR_ACCELERATION, true,
          job::Event::POST_UPDATE);
      }
      requestSpeedChange(target_speed, continuous);
      break;
    }
    case speed_change::Transition::STEP: {
      requestSpeedChange(target_speed, continuous);
      setLinearVelocity(target_speed);
      break;
    }
  }
}

void EntityBase::requestClearRoute()
{
  THROW_SEMANTIC_ERROR(
    "requestClearRoute is only supported for EgoEntity. The specified Entity is not an EgoEntity. "
    "Please check the scenario carefully.");
}

void EntityBase::requestSpeedChangeWithTimeConstraint(
  const double target_speed, const speed_change::Transition transition, double acceleration_time)
{
  if (isTargetSpeedReached(target_speed)) {
    return;
  }
  if (
    std::abs(acceleration_time) <= std::numeric_limits<double>::epsilon() &&
    transition != speed_change::Transition::STEP) {
    requestSpeedChangeWithTimeConstraint(
      target_speed, speed_change::Transition::STEP, acceleration_time);
    return;
  }
  switch (transition) {
    case speed_change::Transition::LINEAR: {
      requestSpeedChangeWithConstantAcceleration(
        target_speed, transition, (target_speed - getCurrentTwist().linear.x) / acceleration_time,
        false);
      break;
    }
    case speed_change::Transition::AUTO: {
      setDynamicConstraints(speed_planner_->planConstraintsFromJerkAndTimeConstraint(
        target_speed, getCurrentTwist(), getCurrentAccel(), acceleration_time,
        getDynamicConstraints()));
      job_list_.append(
        /**
         * @brief Checking if the entity reaches target speed.
         */
        [this, target_speed](double) {
          /**
           * @brief A value of 0.01 is the allowable range for determination of
           * target speed attainment. This value was determined heuristically
           * rather than for technical reasons.
           */
          return std::abs(getCurrentTwist().linear.x - target_speed) < 0.01;
        },
        /**
         * @brief Resets deceleration/acceleration limit.
         */
        [this]() { resetDynamicConstraints(); }, job::Type::LINEAR_ACCELERATION, true,
        job::Event::POST_UPDATE);
      requestSpeedChange(target_speed, false);
      break;
    }
    case speed_change::Transition::STEP: {
      requestSpeedChange(target_speed, false);
      setLinearVelocity(target_speed);
      break;
    }
  }
}

void EntityBase::requestSpeedChange(
  const double target_speed, const speed_change::Transition transition,
  const speed_change::Constraint constraint, const bool continuous)
{
  if (!continuous && isTargetSpeedReached(target_speed)) {
    return;
  }
  switch (constraint.type) {
    case speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION:
      requestSpeedChangeWithConstantAcceleration(
        target_speed, transition, constraint.value, continuous);
      break;
    case speed_change::Constraint::Type::TIME:
      requestSpeedChangeWithTimeConstraint(target_speed, transition, constraint.value);
      break;
    case speed_change::Constraint::Type::NONE:
      requestSpeedChange(target_speed, continuous);
      break;
  }
}

void EntityBase::requestSpeedChangeWithConstantAcceleration(
  const speed_change::RelativeTargetSpeed & target_speed, const speed_change::Transition transition,
  double acceleration, const bool continuous)
{
  if (!continuous && isTargetSpeedReached(target_speed)) {
    return;
  }
  switch (transition) {
    case speed_change::Transition::LINEAR: {
      setLinearAcceleration(acceleration);
      requestSpeedChangeWithConstantAcceleration(
        target_speed, speed_change::Transition::AUTO, acceleration, continuous);
      break;
    }
    case speed_change::Transition::AUTO: {
      job_list_.append(
        /**
         * @brief Checking if the entity reaches target speed.
         */
        [this, target_speed, acceleration](double) {
          double diff =
            target_speed.getAbsoluteValue(getStatus(), other_status_) - getCurrentTwist().linear.x;
          /**
           * @brief Hard coded parameter, threshold for difference
           */
          if (std::abs(diff) <= 0.1) {
            return true;
          }
          if (diff > 0) {
            setAccelerationLimit(std::abs(acceleration));
            return false;
          }
          if (diff < 0) {
            setDecelerationLimit(std::abs(acceleration));
            return false;
          }
          return false;
        },
        /**
         * @brief Resets acceleration limit.
         */
        [this]() { resetDynamicConstraints(); }, job::Type::LINEAR_ACCELERATION, true,
        job::Event::POST_UPDATE);
      requestSpeedChange(target_speed, continuous);
      break;
    }
    case speed_change::Transition::STEP: {
      requestSpeedChange(target_speed, continuous);
      setLinearVelocity(target_speed.getAbsoluteValue(getStatus(), other_status_));
      break;
    }
  }
}

void EntityBase::requestSpeedChangeWithTimeConstraint(
  const speed_change::RelativeTargetSpeed & target_speed, const speed_change::Transition transition,
  double acceleration_time)
{
  if (isTargetSpeedReached(target_speed)) {
    return;
  }
  if (
    std::abs(acceleration_time) <= std::numeric_limits<double>::epsilon() &&
    transition != speed_change::Transition::STEP) {
    requestSpeedChangeWithTimeConstraint(
      target_speed, speed_change::Transition::STEP, acceleration_time);
    return;
  }
  switch (transition) {
    case speed_change::Transition::LINEAR: {
      requestSpeedChangeWithTimeConstraint(
        target_speed.getAbsoluteValue(getStatus(), other_status_), transition, acceleration_time);
      break;
    }
    case speed_change::Transition::AUTO: {
      requestSpeedChangeWithTimeConstraint(
        target_speed.getAbsoluteValue(getStatus(), other_status_), transition, acceleration_time);
      break;
    }
    case speed_change::Transition::STEP: {
      requestSpeedChange(target_speed, false);
      setLinearVelocity(target_speed.getAbsoluteValue(getStatus(), other_status_));
      break;
    }
  }
}

void EntityBase::requestSpeedChange(
  const speed_change::RelativeTargetSpeed & target_speed, const speed_change::Transition transition,
  const speed_change::Constraint constraint, const bool continuous)
{
  if (!continuous && isTargetSpeedReached(target_speed)) {
    return;
  }
  switch (constraint.type) {
    case speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION:
      requestSpeedChangeWithConstantAcceleration(
        target_speed, transition, constraint.value, continuous);
      break;
    case speed_change::Constraint::Type::TIME:
      if (continuous) {
        THROW_SEMANTIC_ERROR("continuous = true is not allowed with time constraint.");
      }
      requestSpeedChangeWithTimeConstraint(target_speed, transition, constraint.value);
      break;
    case speed_change::Constraint::Type::NONE:
      requestSpeedChange(target_speed, continuous);
      break;
  }
}

void EntityBase::requestSpeedChange(double target_speed, bool continuous)
{
  if (!continuous && isTargetSpeedReached(target_speed)) {
    return;
  }
  if (continuous) {
    target_speed_ = target_speed;
    job_list_.append(
      /**
       * @brief If the target entity reaches the target speed, return true.
       */
      [this, target_speed](double) {
        target_speed_ = target_speed;
        return false;
      },
      /**
       * @brief Cancel speed change request.
       */
      [this]() {}, job::Type::LINEAR_VELOCITY, true, job::Event::POST_UPDATE);
  } else {
    target_speed_ = target_speed;
    job_list_.append(
      /**
       * @brief If the target entity reaches the target speed, return true.
       */
      [this, target_speed](double) {
        if (isTargetSpeedReached(target_speed)) {
          return true;
        }
        target_speed_ = target_speed;
        return false;
      },
      /**
       * @brief Cancel speed change request.
       */
      [this]() { target_speed_ = std::nullopt; }, job::Type::LINEAR_VELOCITY, true,
      job::Event::POST_UPDATE);
  }
}

void EntityBase::requestSpeedChange(
  const speed_change::RelativeTargetSpeed & target_speed, bool continuous)
{
  if (!continuous && isTargetSpeedReached(target_speed)) {
    return;
  }
  if (continuous) {
    job_list_.append(
      /**
       * @brief If the target entity reaches the target speed, return true.
       */
      [this, target_speed](double) {
        if (other_status_.find(target_speed.reference_entity_name) == other_status_.end()) {
          return true;
        }
        target_speed_ = target_speed.getAbsoluteValue(getStatus(), other_status_);
        return false;
      },
      [this]() {}, job::Type::LINEAR_VELOCITY, true, job::Event::POST_UPDATE);
  } else {
    job_list_.append(
      /**
       * @brief If the target entity reaches the target speed, return true.
       */
      [this, target_speed](double) {
        if (other_status_.find(target_speed.reference_entity_name) == other_status_.end()) {
          return true;
        }
        if (isTargetSpeedReached(target_speed)) {
          target_speed_ = target_speed.getAbsoluteValue(getStatus(), other_status_);
          return true;
        }
        return false;
      },
      /**
       * @brief Cancel speed change request.
       */
      [this]() { target_speed_ = std::nullopt; }, job::Type::LINEAR_VELOCITY, true,
      job::Event::POST_UPDATE);
  }
}

auto EntityBase::isControlledBySimulator() const -> bool { return true; }

auto EntityBase::setControlledBySimulator(bool /*unused*/) -> void
{
  THROW_SEMANTIC_ERROR(
    getEntityTypename(), " type entities do not support setControlledBySimulator");
}

auto EntityBase::requestFollowTrajectory(
  const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> &) -> void
{
  THROW_SEMANTIC_ERROR(
    getEntityTypename(), " type entities do not support follow trajectory action.");
}

void EntityBase::requestWalkStraight()
{
  THROW_SEMANTIC_ERROR(getEntityTypename(), " type entities do not support WalkStraightAction");
}

void EntityBase::setDynamicConstraints(
  const traffic_simulator_msgs::msg::DynamicConstraints & constraints)
{
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints = constraints;
  setBehaviorParameter(behavior_parameter);
}

void EntityBase::setOtherStatus(
  const std::unordered_map<std::string, CanonicalizedEntityStatus> & status)
{
  other_status_ = status;
  other_status_.erase(name);
}

auto EntityBase::setStatus(const CanonicalizedEntityStatus & status) -> void
{
  auto new_status = static_cast<EntityStatus>(status);

  /*
     FIXME: DIRTY HACK!!!

     It seems that some operations set an incomplete status without respecting
     the original status obtained by getStatus. Below is the code to compensate
     for the lack of set status.
  */
  new_status.name = name;
  new_status.type = getEntityType();
  new_status.subtype = getEntitySubtype();
  new_status.bounding_box = getBoundingBox();
  new_status.action_status.current_action = getCurrentAction();
  status_ = CanonicalizedEntityStatus(new_status, hdmap_utils_ptr_);
}

auto EntityBase::setLinearVelocity(const double linear_velocity) -> void
{
  auto status = static_cast<EntityStatus>(getStatus());
  status.action_status.twist.linear.x = linear_velocity;
  setStatus(CanonicalizedEntityStatus(status, hdmap_utils_ptr_));
}

auto EntityBase::setLinearAcceleration(const double linear_acceleration) -> void
{
  auto status = static_cast<EntityStatus>(getStatus());
  status.action_status.accel.linear.x = linear_acceleration;
  setStatus(CanonicalizedEntityStatus(status, hdmap_utils_ptr_));
}

void EntityBase::setTrafficLightManager(
  const std::shared_ptr<traffic_simulator::TrafficLightManager> & traffic_light_manager)
{
  traffic_light_manager_ = traffic_light_manager;
}

auto EntityBase::setTwist(const geometry_msgs::msg::Twist & twist) -> void
{
  auto new_status = static_cast<EntityStatus>(getStatus());
  new_status.action_status.twist = twist;
  status_ = CanonicalizedEntityStatus(new_status, hdmap_utils_ptr_);
}

auto EntityBase::setAcceleration(const geometry_msgs::msg::Accel & accel) -> void
{
  auto new_status = static_cast<EntityStatus>(getStatus());
  new_status.action_status.accel = accel;
  status_ = CanonicalizedEntityStatus(new_status, hdmap_utils_ptr_);
}

auto EntityBase::setLinearJerk(const double linear_jerk) -> void
{
  auto new_status = static_cast<EntityStatus>(getStatus());
  new_status.action_status.linear_jerk = linear_jerk;
  status_ = CanonicalizedEntityStatus(new_status, hdmap_utils_ptr_);
}

auto EntityBase::setMapPose(const geometry_msgs::msg::Pose &) -> void
{
  THROW_SEMANTIC_ERROR(
    "You cannot set map pose to the vehicle other than ego named ", std::quoted(name), ".");
}

void EntityBase::activateOutOfRangeJob(
  double min_velocity, double max_velocity, double min_acceleration, double max_acceleration,
  double min_jerk, double max_jerk)
{
  /**
   * @brief This value was determined heuristically rather than for
   * technical reasons.
   */
  constexpr double tolerance = 0.01;
  job_list_.append(
    /**
     * @brief Checking if the values of velocity, acceleration and jerk are within the acceptable
     * range
     */
    [this, tolerance, max_velocity, min_velocity, min_acceleration, max_acceleration, min_jerk,
     max_jerk](double) {
      const auto velocity = getCurrentTwist().linear.x;
      const auto accel = getCurrentAccel().linear.x;
      const auto jerk = getLinearJerk();
      if (!(min_velocity <= velocity + tolerance && velocity - tolerance <= max_velocity)) {
        THROW_SPECIFICATION_VIOLATION(
          "Entity: ", name, " - current velocity (which is ", velocity,
          ") is out of range (which is [", min_velocity, ", ", max_velocity, "])");
      }
      if (!(min_acceleration <= accel + tolerance && accel - tolerance <= max_acceleration)) {
        THROW_SPECIFICATION_VIOLATION(
          "Entity: ", name, " - current acceleration (which is ", accel,
          ") is out of range (which is [", min_acceleration, ", ", max_acceleration, "])");
      }
      if (!(min_jerk <= jerk + tolerance && jerk - tolerance <= max_jerk)) {
        THROW_SPECIFICATION_VIOLATION(
          "Entity: ", name, " - current jerk (which is ", jerk, ") is out of range (which is [",
          min_jerk, ", ", max_jerk, "])");
      }
      return false;
    },
    /**
     * @brief This job is always ACTIVE
     */
    [this]() {}, job::Type::OUT_OF_RANGE, true, job::Event::POST_UPDATE);
}

void EntityBase::startNpcLogic(const double current_time)
{
  updateEntityStatusTimestamp(current_time);
  npc_logic_started_ = true;
}

void EntityBase::stopAtCurrentPosition()
{
  auto status = static_cast<EntityStatus>(getStatus());
  status.action_status.twist = geometry_msgs::msg::Twist();
  status.action_status.accel = geometry_msgs::msg::Accel();
  status.action_status.linear_jerk = 0;
  setStatus(CanonicalizedEntityStatus(status, hdmap_utils_ptr_));
}

void EntityBase::updateEntityStatusTimestamp(const double current_time)
{
  auto status = static_cast<EntityStatus>(getStatus());
  status.time = current_time;
  setStatus(CanonicalizedEntityStatus(status, hdmap_utils_ptr_));
}

auto EntityBase::updateStandStillDuration(const double step_time) -> double
{
  if (
    npc_logic_started_ and
    std::abs(getCurrentTwist().linear.x) <= std::numeric_limits<double>::epsilon()) {
    return stand_still_duration_ += step_time;
  }
  return stand_still_duration_ = 0.0;
}

auto EntityBase::updateTraveledDistance(const double step_time) -> double
{
  if (npc_logic_started_) {
    traveled_distance_ += std::abs(getCurrentTwist().linear.x) * step_time;
  }
  return traveled_distance_;
}

}  // namespace entity
}  // namespace traffic_simulator
