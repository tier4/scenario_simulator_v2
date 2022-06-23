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

#include <limits>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/math/distance.hpp>
#include <traffic_simulator/math/polygon.hpp>
#include <traffic_simulator/math/transform.hpp>
#include <unordered_map>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
EntityBase::EntityBase(
  const std::string & name, const traffic_simulator_msgs::msg::EntitySubtype & subtype)
: name(name), status_(boost::none), verbose_(true), visibility_(true), entity_subtype_(subtype)
{
  status_ = boost::none;
}

void EntityBase::appendDebugMarker(visualization_msgs::msg::MarkerArray & /*marker_array*/)
{
  return;
}

auto EntityBase::asAutoware() const -> concealer::Autoware &
{
  throw common::Error(
    "An operation was requested for Entity ", std::quoted(name),
    " that is valid only for the entity controlled by Autoware, but ", std::quoted(name),
    " is not the entity controlled by Autoware.");
}

void EntityBase::onUpdate(double, double)
{
  job_list_.update();
  status_before_update_ = status_;
}

boost::optional<double> EntityBase::getStandStillDuration() const { return stand_still_duration_; }

void EntityBase::requestSpeedChange(
  const double target_speed, const speed_change::Transition transition,
  const speed_change::Constraint constraint, const bool continuous)
{
  switch (transition) {
    case speed_change::Transition::LINEAR: {
      if (getStatus().action_status.twist.linear.x < target_speed) {
        setAccelerationLimit(std::abs(constraint.value));
        job_list_.append(
          /**
           * @brief Checking if the entity reaches target speed.
           */
          [this, target_speed]() {
            return getStatus().action_status.twist.linear.x >= target_speed;
          },
          /**
           * @brief Resets acceleration limit.
           */
          [this]() {
            setAccelerationLimit(traffic_simulator_msgs::msg::DriverModel().acceleration);
          },
          job::Type::LINEAR_ACCELERATION, true);
      } else if (getStatus().action_status.twist.linear.x > target_speed) {
        setDecelerationLimit(std::abs(constraint.value));
        job_list_.append(
          /**
           * @brief Checking if the entity reaches target speed.
           */
          [this, target_speed]() {
            return getStatus().action_status.twist.linear.x <= target_speed;
          },
          /**
           * @brief Resets deceleration limit.
           */
          [this]() {
            setDecelerationLimit(traffic_simulator_msgs::msg::DriverModel().deceleration);
          },
          job::Type::LINEAR_ACCELERATION, true);
      }
      requestSpeedChange(target_speed, continuous);
      break;
    }
    case speed_change::Transition::STEP: {
      auto status = getStatus();
      status.action_status.twist.linear.x = target_speed;
      requestSpeedChange(target_speed, continuous);
      setStatus(status);
      break;
    }
  }
}

void EntityBase::requestSpeedChange(
  const speed_change::RelativeTargetSpeed & target_speed, const speed_change::Transition transition,
  const speed_change::Constraint constraint, const bool continuous)
{
  switch (transition) {
    case speed_change::Transition::LINEAR: {
      job_list_.append(
        /**
           * @brief Checking if the entity reaches target speed.
           */
        [this, target_speed, constraint]() {
          double diff =
            target_speed.getAbsoluteValue(other_status_) - getStatus().action_status.twist.linear.x;
          /**
           * @brief Hard coded parameter, threashold for difference
           */
          if (std::abs(diff) <= 0.1) {
            return true;
          }
          if (diff > 0) {
            setAccelerationLimit(std::abs(constraint.value));
            return false;
          }
          if (diff < 0) {
            setDecelerationLimit(std::abs(constraint.value));
            return false;
          }
          return false;
        },
        /**
           * @brief Resets acceleration limit.
           */
        [this]() { setAccelerationLimit(traffic_simulator_msgs::msg::DriverModel().acceleration); },
        job::Type::LINEAR_ACCELERATION, true);
      requestSpeedChange(target_speed, continuous);
      break;
    }
    case speed_change::Transition::STEP: {
      auto status = getStatus();
      status.action_status.twist.linear.x = target_speed.getAbsoluteValue(other_status_);
      requestSpeedChange(target_speed, continuous);
      setStatus(status);
      break;
    }
  }
}

void EntityBase::requestSpeedChange(double target_speed, bool continuous)
{
  if (continuous) {
    job_list_.append(
      /**
       * @brief If the target entity reaches the target speed, return true.
       */
      [this, target_speed]() {
        target_speed_ = target_speed;
        return false;
      },
      /**
       * @brief Cansel speed change request.
       */
      [this]() {}, job::Type::LINEAR_VELOCITY, true);
  } else {
    job_list_.append(
      /**
       * @brief If the target entity reaches the target speed, return true.
       */
      [this, target_speed]() {
        if (getStatus().action_status.twist.linear.x >= target_speed) {
          return true;
        }
        target_speed_ = target_speed;
        return false;
      },
      /**
       * @brief Cansel speed change request.
       */
      [this]() { target_speed_ = boost::none; }, job::Type::LINEAR_VELOCITY, true);
  }
}

void EntityBase::requestSpeedChange(
  const speed_change::RelativeTargetSpeed & target_speed, bool continuous)
{
  if (continuous) {
    job_list_.append(
      /**
       * @brief If the target entity reaches the target speed, return true.
       */
      [this, target_speed]() {
        if (other_status_.find(target_speed.reference_entity_name) == other_status_.end()) {
          return true;
        }
        target_speed_ = target_speed.getAbsoluteValue(other_status_);
        return false;
      },
      [this]() {}, job::Type::LINEAR_VELOCITY, true);
  } else {
    job_list_.append(
      /**
       * @brief If the target entity reaches the target speed, return true.
       */
      [this, target_speed]() {
        if (other_status_.find(target_speed.reference_entity_name) == other_status_.end()) {
          return true;
        }
        if (
          getStatus().action_status.twist.linear.x >=
          target_speed.getAbsoluteValue(other_status_)) {
          target_speed_ = target_speed.getAbsoluteValue(other_status_);
          return true;
        }
        return false;
      },
      /**
       * @brief Cansel speed change request.
       */
      [this]() { target_speed_ = boost::none; }, job::Type::LINEAR_VELOCITY, true);
  }
}

void EntityBase::requestLaneChange(
  const traffic_simulator::lane_change::AbsoluteTarget & target,
  const traffic_simulator::lane_change::TrajectoryShape trajectory_shape,
  const traffic_simulator::lane_change::Constraint & constraint)
{
  auto param = traffic_simulator::lane_change::Parameter(target, trajectory_shape, constraint);
  requestLaneChange(param);
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
      traffic_simulator::lane_change::AbsoluteTarget(lane_change_target_id.get(), target.offset),
      trajectory_shape, constraint);
  } else {
    THROW_SEMANTIC_ERROR(
      "Failed to calculate absolute target lane. Please check the target lane exists.");
  }
}

void EntityBase::updateStandStillDuration(const double step_time)
{
  if (!status_) {
    stand_still_duration_ = boost::none;
  } else {
    if (!stand_still_duration_) {
      stand_still_duration_ = 0;
    }
    if (
      std::fabs(status_->action_status.twist.linear.x) <= std::numeric_limits<double>::epsilon()) {
      stand_still_duration_ = step_time + stand_still_duration_.get();
    } else {
      stand_still_duration_ = 0;
    }
  }
}

void EntityBase::updateEntityStatusTimestamp(const double current_time)
{
  if (status_) {
    status_->time = current_time;
  }
}

void EntityBase::setOtherStatus(
  const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityStatus> & status)
{
  other_status_.clear();
  if (status_) {
    for (const auto & each : status) {
      if (each.first != name) {
        const auto p0 = each.second.pose.position;
        const auto p1 = status_.get().pose.position;
        double distance =
          std::sqrt(std::pow(p0.x - p1.x, 2) + std::pow(p0.y - p1.y, 2) + std::pow(p0.z - p1.z, 2));
        if (distance < 30) {
          other_status_.insert(each);
        }
      }
    }
  }
}

const traffic_simulator_msgs::msg::EntityStatus EntityBase::getStatus() const
{
  if (!status_) {
    THROW_SEMANTIC_ERROR("status is not set");
  } else {
    auto status = this->status_.get();
    status.bounding_box = getBoundingBox();
    status.subtype = entity_subtype_;
    status.type = entity_type_;
    return status;
  }
}

bool EntityBase::setStatus(const traffic_simulator_msgs::msg::EntityStatus & status)
{
  status_ = status;
  status_->name = name;
  return true;
}

auto EntityBase::getMapPose() const -> geometry_msgs::msg::Pose
{
  const auto status = getStatus();
  return status.pose;
}

auto EntityBase::getMapPose(const geometry_msgs::msg::Pose & relative_pose)
  -> geometry_msgs::msg::Pose
{
  const auto ref_status = getStatus();
  tf2::Transform ref_transform, relative_transform;
  tf2::fromMsg(ref_status.pose, ref_transform);
  tf2::fromMsg(relative_pose, relative_transform);
  geometry_msgs::msg::Pose ret;
  tf2::toMsg(ref_transform * relative_transform, ret);
  return ret;
}

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

  return math::get2DConvexHull(points_bbox);
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
  const auto bound = hdmap_utils_ptr_->getLeftBound(lanelet_id);
  if (bound.empty()) {
    THROW_SEMANTIC_ERROR(
      "Failed to calculate left bounds of lanelet_id : ", lanelet_id, " please check lanelet map.");
  }
  const auto polygon = math::transformPoints(getMapPose(), get2DPolygon());
  if (polygon.empty()) {
    THROW_SEMANTIC_ERROR(
      "Failed to calculate 2d polygon of entity: ", name, " . Please check ", name,
      " exists and it's definition");
  }
  /*
  for (const auto & p : bound) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("bound"), p.x << "," << p.y << "," << p.z);
  }
  for (const auto & p : polygon) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("polygon"), p.x << "," << p.y << "," << p.z);
  }
  */
  return math::getDistance2D(bound, polygon);
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
  const auto bound = hdmap_utils_ptr_->getRightBound(lanelet_id);
  if (bound.empty()) {
    THROW_SEMANTIC_ERROR(
      "Failed to calculate right bounds of lanelet_id : ", lanelet_id,
      " please check lanelet map.");
  }
  const auto polygon = math::transformPoints(getMapPose(), get2DPolygon());
  if (polygon.empty()) {
    THROW_SEMANTIC_ERROR(
      "Failed to calculate 2d polygon of entity: ", name, " . Please check ", name,
      " exists and it's definition");
  }
  return math::getDistance2D(bound, polygon);
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

void EntityBase::stopAtEndOfRoad()
{
  if (!status_) {
    THROW_SEMANTIC_ERROR("status is not set");
  } else {
    status_.get().action_status.twist = geometry_msgs::msg::Twist();
    status_.get().action_status.accel = geometry_msgs::msg::Accel();
  }
}

void EntityBase::setAccelerationLimit(double)
{
  THROW_SIMULATION_ERROR(
    "setAccelerationLimit function can be used with only ego/vehicle/pedestrian entity.");
}

void EntityBase::setDecelerationLimit(double)
{
  THROW_SIMULATION_ERROR(
    "setAccelerationLimit function can be used with only ego/vehicle/pedestrian entity.");
}

auto EntityBase::getLaneletPose() const -> boost::optional<traffic_simulator_msgs::msg::LaneletPose>
{
  const auto status = getStatus();
  if (status.lanelet_pose_valid) {
    return status.lanelet_pose;
  }
  if (getEntityType().type == traffic_simulator_msgs::msg::EntityType::VEHICLE) {
    return hdmap_utils_ptr_->toLaneletPose(status.pose, getBoundingBox(), false);
  } else {
    return hdmap_utils_ptr_->toLaneletPose(status.pose, getBoundingBox(), true);
  }
}
}  // namespace entity
}  // namespace traffic_simulator
