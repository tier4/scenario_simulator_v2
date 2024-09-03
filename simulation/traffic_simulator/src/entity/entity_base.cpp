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
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator/utils/pose.hpp>
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
  status_(std::make_shared<CanonicalizedEntityStatus>(entity_status)),
  status_before_update_(*status_),
  hdmap_utils_ptr_(hdmap_utils_ptr)
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

auto EntityBase::getCanonicalizedLaneletPose() const -> std::optional<CanonicalizedLaneletPose>
{
  return status_->getCanonicalizedLaneletPose();
}

auto EntityBase::getCanonicalizedLaneletPose(double matching_distance) const
  -> std::optional<CanonicalizedLaneletPose>
{
  const auto include_crosswalk = [](const auto & entity_type) {
    return (traffic_simulator_msgs::msg::EntityType::PEDESTRIAN == entity_type.type) ||
           (traffic_simulator_msgs::msg::EntityType::MISC_OBJECT == entity_type.type);
  }(getEntityType());

  // prefer the current lanelet
  return pose::toCanonicalizedLaneletPose(
    status_->getMapPose(), status_->getBoundingBox(), status_->getLaneletIds(), include_crosswalk,
    matching_distance, hdmap_utils_ptr_);
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
  return isTargetSpeedReached(
    target_speed.getAbsoluteValue(getCanonicalizedStatus(), other_status_));
}

auto EntityBase::onUpdate(const double /*current_time*/, const double step_time) -> void
{
  job_list_.update(step_time, job::Event::PRE_UPDATE);
  step_time_ = step_time;
  status_before_update_.set(*status_);
  speed_planner_ =
    std::make_unique<traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner>(
      step_time, name);
}

auto EntityBase::onPostUpdate(const double /*current_time*/, const double step_time) -> void
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
  if (target.entity_name == name) {
    if (not laneMatchingSucceed()) {
      THROW_SEMANTIC_ERROR(
        "Source entity does not assigned to lanelet. Please check source entity name : ", name,
        " exists on lane.");
    }
    reference_lanelet_id = status_->getLaneletId();
  } else {
    if (other_status_.find(target.entity_name) == other_status_.end()) {
      THROW_SEMANTIC_ERROR(
        "Target entity : ", target.entity_name, " does not exist. Please check ",
        target.entity_name, " exists.");
    } else if (!other_status_.at(target.entity_name).laneMatchingSucceed()) {
      THROW_SEMANTIC_ERROR(
        "Target entity does not assigned to lanelet. Please check Target entity name : ",
        target.entity_name, " exists on lane.");
    } else {
      reference_lanelet_id = other_status_.at(target.entity_name).getLaneletId();
    }
  }

  if (
    const auto lane_change_target_id = hdmap_utils_ptr_->getLaneChangeableLaneletId(
      reference_lanelet_id, target.direction, target.shift)) {
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
          double diff = target_speed.getAbsoluteValue(getCanonicalizedStatus(), other_status_) -
                        getCurrentTwist().linear.x;
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
      setLinearVelocity(target_speed.getAbsoluteValue(getCanonicalizedStatus(), other_status_));
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
        target_speed.getAbsoluteValue(getCanonicalizedStatus(), other_status_), transition,
        acceleration_time);
      break;
    }
    case speed_change::Transition::AUTO: {
      requestSpeedChangeWithTimeConstraint(
        target_speed.getAbsoluteValue(getCanonicalizedStatus(), other_status_), transition,
        acceleration_time);
      break;
    }
    case speed_change::Transition::STEP: {
      requestSpeedChange(target_speed, false);
      setLinearVelocity(target_speed.getAbsoluteValue(getCanonicalizedStatus(), other_status_));
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
        target_speed_ = target_speed.getAbsoluteValue(getCanonicalizedStatus(), other_status_);
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
          target_speed_ = target_speed.getAbsoluteValue(getCanonicalizedStatus(), other_status_);
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

auto EntityBase::setStatus(const EntityStatus & status, const lanelet::Ids & lanelet_ids) -> void
{
  status_->set(
    status, lanelet_ids, getDefaultMatchingDistanceForLaneletPoseCalculation(), hdmap_utils_ptr_);
}

auto EntityBase::setStatus(const EntityStatus & status) -> void
{
  status_->set(status, getDefaultMatchingDistanceForLaneletPoseCalculation(), hdmap_utils_ptr_);
}

auto EntityBase::setCanonicalizedStatus(const CanonicalizedEntityStatus & status) -> void
{
  status_->set(status);
}

auto EntityBase::setLinearVelocity(const double linear_velocity) -> void
{
  status_->setLinearVelocity(linear_velocity);
}

auto EntityBase::setLinearAcceleration(const double linear_acceleration) -> void
{
  status_->setLinearAcceleration(linear_acceleration);
}

void EntityBase::setTrafficLightManager(
  const std::shared_ptr<traffic_simulator::TrafficLightManager> & traffic_light_manager)
{
  traffic_light_manager_ = traffic_light_manager;
}

auto EntityBase::setTwist(const geometry_msgs::msg::Twist & twist) -> void
{
  status_->setTwist(twist);
}

auto EntityBase::setAcceleration(const geometry_msgs::msg::Accel & accel) -> void
{
  status_->setAccel(accel);
}

auto EntityBase::setLinearJerk(const double linear_jerk) -> void
{
  status_->setLinearJerk(linear_jerk);
}

auto EntityBase::setAction(const std::string & action) -> void { status_->setAction(action); }

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

void EntityBase::stopAtCurrentPosition()
{
  status_->setTwist(geometry_msgs::msg::Twist());
  status_->setAccel(geometry_msgs::msg::Accel());
  status_->setLinearJerk(0.0);
}

void EntityBase::updateEntityStatusTimestamp(const double current_time)
{
  status_->setTime(current_time);
}

auto EntityBase::updateStandStillDuration(const double step_time) -> double
{
  if (std::abs(getCurrentTwist().linear.x) <= std::numeric_limits<double>::epsilon()) {
    return stand_still_duration_ += step_time;
  } else {
    return stand_still_duration_ = 0.0;
  }
}

auto EntityBase::updateTraveledDistance(const double step_time) -> double
{
  return traveled_distance_ += std::abs(getCurrentTwist().linear.x) * step_time;
}

bool EntityBase::reachPosition(const std::string & target_name, const double tolerance) const
{
  return reachPosition(other_status_.find(target_name)->second.getMapPose(), tolerance);
}

bool EntityBase::reachPosition(
  const geometry_msgs::msg::Pose & target_pose, const double tolerance) const
{
  return math::geometry::getDistance(getMapPose(), target_pose) < tolerance;
}

bool EntityBase::reachPosition(
  const CanonicalizedLaneletPose & lanelet_pose, const double tolerance) const
{
  return reachPosition(static_cast<geometry_msgs::msg::Pose>(lanelet_pose), tolerance);
}

/***
 * @brief Request synchronize the entity with the target entity.
 * @param target_name The name of the target entity.
 * @param target_sync_pose The target lanelet pose of the target entity.
 * @param entity_target The target lanelet pose of the entity to control.
 * @param target_speed The target velocity of the entity to control.
 * @param tolerance The threshold to determine if the entity has already arrived to the target lanelet.
*/

auto EntityBase::requestSynchronize(
  const std::string & target_name, const CanonicalizedLaneletPose & target_sync_pose,
  const CanonicalizedLaneletPose & entity_target, const double target_speed, const double tolerance)
  -> bool
{
  if (traffic_simulator_msgs::msg::EntityType::EGO == getEntityType().type) {
    THROW_SYNTAX_ERROR("Request synchronize is only for non-ego entities.");
  }

  if (tolerance == 0) {
    RCLCPP_WARN_ONCE(
      rclcpp::get_logger("traffic_simulator"),
      "The tolerance is set to 0.0. This may cause the entity to never reach the target lanelet.");
  }

  ///@brief Check if the entity has already arrived to the target lanelet.
  if (reachPosition(entity_target, tolerance)) {
    if (getCurrentTwist().linear.x < target_speed + getMaxAcceleration() * step_time_) {
    } else {
      RCLCPP_WARN_ONCE(
        rclcpp::get_logger("traffic_simulator"),
        "The entity has already arrived to the target lanelet and the entity is not nearly "
        "stopped.");
    }
    target_speed_ = target_speed;
    return true;
  }

  job_list_.append(
    [this, target_name, target_sync_pose, entity_target, target_speed](double) {
      const auto entity_lanelet_pose = getCanonicalizedLaneletPose();
      if (!entity_lanelet_pose.has_value()) {
        THROW_SEMANTIC_ERROR(
          "Failed to get lanelet pose of the entity. Check if the entity is on the lane."
          "If so please contact the developer since there might be an undiscovered bug.");
      }

      const auto entity_distance = longitudinalDistance(
        entity_lanelet_pose.value(), entity_target, true, true, true, hdmap_utils_ptr_);
      if (!entity_distance.has_value()) {
        THROW_SEMANTIC_ERROR(
          "Failed to get distance between entity and target lanelet pose. Check if the entity has "
          "already passed the target lanelet. If not, please contact the developer since there "
          "might be an undiscovered bug.");
      }

      const auto target_entity_lanelet_pose =
        other_status_.find(target_name) == other_status_.end()
          ? THROW_SEMANTIC_ERROR("Failed to find target entity. Check if the target entity exists.")
          : other_status_.find(target_name)->second.getLaneletPose();

      const auto target_entity_distance = longitudinalDistance(
        CanonicalizedLaneletPose(target_entity_lanelet_pose, hdmap_utils_ptr_), target_sync_pose,
        true, true, true, hdmap_utils_ptr_);
      if (!target_entity_distance.has_value() || target_entity_distance.value() < 0) {
        RCLCPP_WARN_ONCE(
          rclcpp::get_logger("traffic_simulator"),
          "Failed to get distance between target entity and target lanelet pose. Check if target "
          "entity has already passed the target lanelet. If not, please contact the developer "
          "since there might be an undiscovered bug.");
        return true;
      }

      const auto target_entity_velocity =
        other_status_.find(target_name)->second.getTwist().linear.x;
      const auto entity_velocity = getCurrentTwist().linear.x;
      const auto target_entity_arrival_time =
        (std::abs(target_entity_velocity) > std::numeric_limits<double>::epsilon())
          ? target_entity_distance.value() / target_entity_velocity
          : 0;

      auto entity_velocity_to_synchronize = [this, entity_velocity, target_entity_arrival_time,
                                             entity_distance, target_speed]() {
        const auto border_distance =
          (entity_velocity + target_speed) * target_entity_arrival_time / 2;
        if (border_distance < entity_distance.value()) {
          ///@brief Making entity speed up.
          return entity_velocity + getMaxAcceleration() * step_time_;
        } else if (border_distance > entity_distance.value()) {
          ///@brief Making entity slow down.
          return entity_velocity - getMaxDeceleration() * step_time_;
        } else {
          ///@brief Making entity keep the current speed.
          return entity_velocity;
        }
      };

      /**
       * @warning using this->requestSpeedChange here does not work in some kind of reason.
       * It seems that after this, function is called by some reason. func_on_cleanup will be deleted and becomes nullptr
       */
      target_speed_ = entity_velocity_to_synchronize();
      return false;
    },
    [this]() {}, job::Type::LINEAR_ACCELERATION, true, job::Event::POST_UPDATE);
  return false;
}

}  // namespace entity
}  // namespace traffic_simulator
