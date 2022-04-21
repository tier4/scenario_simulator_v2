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

#include <limits>
#include <queue>
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
  const std::string & name, const traffic_simulator_msgs::msg::EntitySubtype & subtype)
: name(name), status_(boost::none), verbose_(true), visibility_(true), entity_subtype_(subtype)
{
  status_ = boost::none;
  job_list_.append(
    /**
     * @brief Update stand still duration.
     */
    [this]() {
      if (!status_) {
        stand_still_duration_ = boost::none;
      } else {
        if (!stand_still_duration_) {
          stand_still_duration_ = 0;
        }
        if (
          std::fabs(status_->action_status.twist.linear.x) <=
          std::numeric_limits<double>::epsilon()) {
          stand_still_duration_ = step_time_ + stand_still_duration_.get();
        } else {
          stand_still_duration_ = 0;
        }
      }
      return false;
    },
    [this]() {}, job::Type::STAND_STILL_DURATION, false);
}

void EntityBase::appendDebugMarker(visualization_msgs::msg::MarkerArray & /*marker_array*/)
{
  return;
}

void EntityBase::onUpdate(double current_time, double step_time)
{
  current_time_ = current_time;
  step_time_ = step_time;
  status_before_update_ = status_;
}

void EntityBase::updateJobList() { job_list_.update(); }

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

auto EntityBase::getVehicleCommand() const -> std::tuple<
  autoware_auto_control_msgs::msg::AckermannControlCommand,
  autoware_auto_vehicle_msgs::msg::GearCommand>
{
  THROW_SIMULATION_ERROR(
    "`getVehicleCommand` is not provided for ", getEntityTypename(), " type entity.");
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
}  // namespace entity
}  // namespace traffic_simulator
