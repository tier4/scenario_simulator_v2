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

#include <geometry/distance.hpp>
#include <geometry/polygon/polygon.hpp>
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
  const std::string & name, const traffic_simulator_msgs::msg::EntityStatus & entity_status)
: name(name), verbose(true), status_(entity_status), status_before_update_(status_)
{
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
  const auto status = getStatus();

  std::vector<geometry_msgs::msg::Point> points_bbox;
  geometry_msgs::msg::Point p0, p1, p2, p3, p4, p5, p6, p7;

  p0.x = status.bounding_box.center.x + status.bounding_box.dimensions.x * 0.5;
  p0.y = status.bounding_box.center.y + status.bounding_box.dimensions.y * 0.5;
  p0.z = status.bounding_box.center.z + status.bounding_box.dimensions.z * 0.5;
  points_bbox.emplace_back(p0);

  p1.x = status.bounding_box.center.x + status.bounding_box.dimensions.x * 0.5;
  p1.y = status.bounding_box.center.y + status.bounding_box.dimensions.y * 0.5;
  p1.z = status.bounding_box.center.z - status.bounding_box.dimensions.z * 0.5;
  points_bbox.emplace_back(p1);

  p2.x = status.bounding_box.center.x + status.bounding_box.dimensions.x * 0.5;
  p2.y = status.bounding_box.center.y - status.bounding_box.dimensions.y * 0.5;
  p2.z = status.bounding_box.center.z + status.bounding_box.dimensions.z * 0.5;
  points_bbox.emplace_back(p2);

  p3.x = status.bounding_box.center.x - status.bounding_box.dimensions.x * 0.5;
  p3.y = status.bounding_box.center.y + status.bounding_box.dimensions.y * 0.5;
  p3.z = status.bounding_box.center.z + status.bounding_box.dimensions.z * 0.5;
  points_bbox.emplace_back(p3);

  p4.x = status.bounding_box.center.x + status.bounding_box.dimensions.x * 0.5;
  p4.y = status.bounding_box.center.y - status.bounding_box.dimensions.y * 0.5;
  p4.z = status.bounding_box.center.z - status.bounding_box.dimensions.z * 0.5;
  points_bbox.emplace_back(p4);

  p5.x = status.bounding_box.center.x - status.bounding_box.dimensions.x * 0.5;
  p5.y = status.bounding_box.center.y + status.bounding_box.dimensions.y * 0.5;
  p5.z = status.bounding_box.center.z - status.bounding_box.dimensions.z * 0.5;
  points_bbox.emplace_back(p5);

  p6.x = status.bounding_box.center.x - status.bounding_box.dimensions.x * 0.5;
  p6.y = status.bounding_box.center.y - status.bounding_box.dimensions.y * 0.5;
  p6.z = status.bounding_box.center.z + status.bounding_box.dimensions.z * 0.5;
  points_bbox.emplace_back(p6);

  p7.x = status.bounding_box.center.x - status.bounding_box.dimensions.x * 0.5;
  p7.y = status.bounding_box.center.y - status.bounding_box.dimensions.y * 0.5;
  p7.z = status.bounding_box.center.z - status.bounding_box.dimensions.z * 0.5;
  points_bbox.emplace_back(p7);

  return math::geometry::get2DConvexHull(points_bbox);
}

auto EntityBase::getCurrentAccel() const -> geometry_msgs::msg::Accel
{
  return getStatus().action_status.accel;
}

auto EntityBase::getCurrentTwist() const -> geometry_msgs::msg::Twist
{
  return getStatus().action_status.twist;
}

auto EntityBase::getDistanceToLaneBound() -> double
{
  return std::min(getDistanceToLeftLaneBound(), getDistanceToRightLaneBound());
}

auto EntityBase::getDistanceToLaneBound(std::int64_t lanelet_id) const -> double
{
  return std::min(getDistanceToLeftLaneBound(lanelet_id), getDistanceToRightLaneBound(lanelet_id));
}

auto EntityBase::getDistanceToLaneBound(const std::vector<std::int64_t> & lanelet_ids) const
  -> double
{
  return std::min(
    getDistanceToLeftLaneBound(lanelet_ids), getDistanceToRightLaneBound(lanelet_ids));
}

auto EntityBase::getDistanceToLeftLaneBound() -> double
{
  return getDistanceToLeftLaneBound(getRouteLanelets());
}

auto EntityBase::getDistanceToLeftLaneBound(std::int64_t lanelet_id) const -> double
{
  if (const auto bound = hdmap_utils_ptr_->getLeftBound(lanelet_id); bound.empty()) {
    THROW_SEMANTIC_ERROR(
      "Failed to calculate left bounds of lanelet_id : ", lanelet_id, " please check lanelet map.");
  } else if (const auto polygon = math::geometry::transformPoints(getMapPose(), get2DPolygon());
             polygon.empty()) {
    THROW_SEMANTIC_ERROR(
      "Failed to calculate 2d polygon of entity: ", name, " . Please check ", name,
      " exists and it's definition");
  } else {
    return math::geometry::getDistance2D(bound, polygon);
  }
}

auto EntityBase::getDistanceToLeftLaneBound(const std::vector<std::int64_t> & lanelet_ids) const
  -> double
{
  std::vector<double> distances;
  std::transform(
    lanelet_ids.begin(), lanelet_ids.end(), std::back_inserter(distances),
    [this](std::int64_t lanelet_id) { return getDistanceToLeftLaneBound(lanelet_id); });
  return *std::min_element(distances.begin(), distances.end());
}

auto EntityBase::getDistanceToRightLaneBound() -> double
{
  return getDistanceToRightLaneBound(getRouteLanelets());
}

auto EntityBase::getDistanceToRightLaneBound(std::int64_t lanelet_id) const -> double
{
  if (const auto bound = hdmap_utils_ptr_->getRightBound(lanelet_id); bound.empty()) {
    THROW_SEMANTIC_ERROR(
      "Failed to calculate right bounds of lanelet_id : ", lanelet_id,
      " please check lanelet map.");
  } else if (const auto polygon = math::geometry::transformPoints(getMapPose(), get2DPolygon());
             polygon.empty()) {
    THROW_SEMANTIC_ERROR(
      "Failed to calculate 2d polygon of entity: ", name, " . Please check ", name,
      " exists and it's definition");
  } else {
    return math::geometry::getDistance2D(bound, polygon);
  }
}

auto EntityBase::getDistanceToRightLaneBound(const std::vector<std::int64_t> & lanelet_ids) const
  -> double
{
  std::vector<double> distances;
  std::transform(
    lanelet_ids.begin(), lanelet_ids.end(), std::back_inserter(distances),
    [this](std::int64_t lanelet_id) { return getDistanceToLeftLaneBound(lanelet_id); });
  return *std::min_element(distances.begin(), distances.end());
}

auto EntityBase::getDynamicConstraints() const
  -> const traffic_simulator_msgs::msg::DynamicConstraints
{
  return getBehaviorParameter().dynamic_constraints;
}

auto EntityBase::getEntityStatusBeforeUpdate() const
  -> const traffic_simulator_msgs::msg::EntityStatus &
{
  return status_before_update_;
}

auto EntityBase::getLinearJerk() const -> double { return getStatus().action_status.linear_jerk; }

auto EntityBase::getLaneletPose() const -> std::optional<traffic_simulator_msgs::msg::LaneletPose>
{
  if (status_.lanelet_pose_valid) {
    return status_.lanelet_pose;
  }
  return std::nullopt;
}

auto EntityBase::getLaneletPose(double matching_distance) const
  -> std::optional<traffic_simulator_msgs::msg::LaneletPose>
{
  if (traffic_simulator_msgs::msg::EntityType::PEDESTRIAN == getStatus().type.type) {
    return hdmap_utils_ptr_->toLaneletPose(
      getMapPose(), getStatus().bounding_box, true, matching_distance);
  } else {
    return hdmap_utils_ptr_->toLaneletPose(
      getMapPose(), getStatus().bounding_box, false, matching_distance);
  }
}

auto EntityBase::fillLaneletPose(
  traffic_simulator_msgs::msg::EntityStatus & status, bool include_crosswalk) const -> void
{
  const auto unique_route_lanelets = traffic_simulator::helper::getUniqueValues(getRouteLanelets());

  std::optional<traffic_simulator_msgs::msg::LaneletPose> lanelet_pose;

  if (unique_route_lanelets.empty()) {
    lanelet_pose = hdmap_utils_ptr_->toLaneletPose(
      status.pose, getStatus().bounding_box, include_crosswalk, 1.0);
  } else {
    lanelet_pose = hdmap_utils_ptr_->toLaneletPose(status.pose, unique_route_lanelets, 1.0);
    if (!lanelet_pose) {
      lanelet_pose = hdmap_utils_ptr_->toLaneletPose(
        status.pose, getStatus().bounding_box, include_crosswalk, 1.0);
    }
  }
  if (lanelet_pose) {
    math::geometry::CatmullRomSpline spline(
      hdmap_utils_ptr_->getCenterPoints(lanelet_pose->lanelet_id));
    if (const auto s_value = spline.getSValue(status.pose)) {
      status.pose.position.z = spline.getPoint(s_value.value()).z;
    }
  }

  status.lanelet_pose_valid = static_cast<bool>(lanelet_pose);
  if (status.lanelet_pose_valid) {
    status.lanelet_pose = lanelet_pose.value();
  }
}

auto EntityBase::getMapPose() const -> geometry_msgs::msg::Pose { return getStatus().pose; }

auto EntityBase::getMapPose(const geometry_msgs::msg::Pose & relative_pose)
  -> geometry_msgs::msg::Pose
{
  tf2::Transform ref_transform, relative_transform;
  tf2::fromMsg(getStatus().pose, ref_transform);
  tf2::fromMsg(relative_pose, relative_transform);
  geometry_msgs::msg::Pose ret;
  tf2::toMsg(ref_transform * relative_transform, ret);
  return ret;
}

auto EntityBase::getStatus() const -> const traffic_simulator_msgs::msg::EntityStatus &
{
  return status_;
}

auto EntityBase::getStandStillDuration() const -> double { return stand_still_duration_; }

auto EntityBase::getTraveledDistance() const -> double { return traveled_distance_; }

auto EntityBase::isNpcLogicStarted() const -> bool { return npc_logic_started_; }

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
  std::int64_t reference_lanelet_id = 0;
  if (target.entity_name == name) {
    if (!getStatus().lanelet_pose_valid) {
      THROW_SEMANTIC_ERROR(
        "Target entity does not assigned to lanelet. Please check Target entity name : ",
        target.entity_name, " exists on lane.");
    }
    reference_lanelet_id = getStatus().lanelet_pose.lanelet_id;
  } else {
    if (other_status_.find(target.entity_name) == other_status_.end()) {
      THROW_SEMANTIC_ERROR(
        "Target entity : ", target.entity_name, " does not exist. Please check ",
        target.entity_name, " exists.");
    }
    if (!other_status_.at(target.entity_name).lanelet_pose_valid) {
      THROW_SEMANTIC_ERROR(
        "Target entity does not assigned to lanelet. Please check Target entity name : ",
        target.entity_name, " exists on lane.");
    }
    reference_lanelet_id = other_status_.at(target.entity_name).lanelet_pose.lanelet_id;
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

void EntityBase::setEntityTypeList(
  const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityType> & entity_type_list)
{
  entity_type_list_ = entity_type_list;
}

void EntityBase::setHdMapUtils(const std::shared_ptr<hdmap_utils::HdMapUtils> & ptr)
{
  hdmap_utils_ptr_ = ptr;
}

void EntityBase::setOtherStatus(
  const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityStatus> & status)
{
  other_status_.clear();
  for (const auto & [other_name, other_status] : status) {
    if (other_name != name) {
      /*
         The following filtering is the code written for the purpose of
         reducing the calculation load, but it is commented out experimentally
         because it adversely affects "processing that needs to identify other
         entities regardless of distance" such as RelativeTargetSpeed of
         requestSpeedChange.
      */
      // const auto p0 = other_status.pose.position;
      // const auto p1 = status_.pose.position;
      // if (const auto distance = std::hypot(p0.x - p1.x, p0.y - p1.y, p0.z - p1.z); distance <
      // 30)
      // {
      other_status_.emplace(other_name, other_status);
      // }
    }
  }
}

auto EntityBase::setStatus(const traffic_simulator_msgs::msg::EntityStatus & status) -> void
{
  auto new_status = status;

  /*
     FIXME: DIRTY HACK!!!

     It seems that some operations set an incomplete status without respecting
     the original status obtained by getStatus. Below is the code to compensate
     for the lack of set status.
  */
  new_status.name = name;
  new_status.type = status_.type;
  new_status.subtype = status_.subtype;
  new_status.bounding_box = status_.bounding_box;
  new_status.action_status.current_action = getCurrentAction();

  status_ = new_status;
}

auto EntityBase::setLinearVelocity(const double linear_velocity) -> void
{
  auto status = getStatus();
  status.action_status.twist.linear.x = linear_velocity;
  setStatus(status);
}

auto EntityBase::setLinearAcceleration(const double linear_acceleration) -> void
{
  auto status = getStatus();
  status.action_status.accel.linear.x = linear_acceleration;
  setStatus(status);
}

void EntityBase::setTrafficLightManager(
  const std::shared_ptr<traffic_simulator::TrafficLightManager> & traffic_light_manager)
{
  traffic_light_manager_ = traffic_light_manager;
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
      auto velocity_ = status_.action_status.twist.linear.x;
      auto accel_ = status_.action_status.accel.linear.x;
      auto jerk_ = status_.action_status.linear_jerk;
      if (!(min_velocity <= velocity_ + tolerance && velocity_ - tolerance <= max_velocity)) {
        THROW_SPECIFICATION_VIOLATION(
          "Entity: ", name, " - current velocity (which is ", velocity_,
          ") is out of range (which is [", min_velocity, ", ", max_velocity, "])");
      }
      if (!(min_acceleration <= accel_ + tolerance && accel_ - tolerance <= max_acceleration)) {
        THROW_SPECIFICATION_VIOLATION(
          "Entity: ", name, " - current acceleration (which is ", accel_,
          ") is out of range (which is [", min_acceleration, ", ", max_acceleration, "])");
      }
      if (!(min_jerk <= jerk_ + tolerance && jerk_ - tolerance <= max_jerk)) {
        THROW_SPECIFICATION_VIOLATION(
          "Entity: ", name, " - current jerk (which is ", jerk_, ") is out of range (which is [",
          min_jerk, ", ", max_jerk, "])");
      }
      return false;
    },
    /**
     * @brief This job is always ACTIVE
     */
    [this]() {}, job::Type::OUT_OF_RANGE, true, job::Event::POST_UPDATE);
}

auto EntityBase::setVelocityLimit(double) -> void {}

void EntityBase::startNpcLogic() { npc_logic_started_ = true; }

void EntityBase::stopAtEndOfRoad()
{
  status_.action_status.twist = geometry_msgs::msg::Twist();
  status_.action_status.accel = geometry_msgs::msg::Accel();
  status_.action_status.linear_jerk = 0;
}

void EntityBase::updateEntityStatusTimestamp(const double current_time)
{
  status_.time = current_time;
}

auto EntityBase::updateStandStillDuration(const double step_time) -> double
{
  if (
    npc_logic_started_ and
    std::abs(status_.action_status.twist.linear.x) <= std::numeric_limits<double>::epsilon()) {
    return stand_still_duration_ += step_time;
  } else {
    return stand_still_duration_ = 0.0;
  }
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
