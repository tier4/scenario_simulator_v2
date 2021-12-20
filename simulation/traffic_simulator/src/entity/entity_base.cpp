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
EntityBase::EntityBase(const std::string & type, const std::string & name)
: type(type), name(name), status_(boost::none), verbose_(true), visibility_(true)
{
  status_ = boost::none;
}

void EntityBase::appendDebugMarker(visualization_msgs::msg::MarkerArray & /*marker_array*/)
{
  return;
}

void EntityBase::onUpdate(double, double) { status_before_update_ = status_; }

boost::optional<double> EntityBase::getStandStillDuration() const { return stand_still_duration_; }

const autoware_vehicle_msgs::msg::VehicleCommand EntityBase::getVehicleCommand()
{
  THROW_SIMULATION_ERROR("get vehicle command does not support in ", type, " entity type");
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
    return this->status_.get();
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

auto EntityBase::getDriverModel() -> const traffic_simulator_msgs::msg::DriverModel
{
  THROW_SIMULATION_ERROR("getDriverModel function can be used with only ego/vehicle entity.");
}

void EntityBase::setDriverModel(const traffic_simulator_msgs::msg::DriverModel &)
{
  // THROW_SIMULATION_ERROR("setDriverModel function can be used with only ego/vehicle entity.");
}

void EntityBase::setAccelerationLimit(double)
{
  THROW_SIMULATION_ERROR("setAccelerationLimit function can be used with only ego/vehicle entity.");
}

void EntityBase::setDecelerationLimit(double)
{
  THROW_SIMULATION_ERROR("setAccelerationLimit function can be used with only ego/vehicle entity.");
}
}  // namespace entity
}  // namespace traffic_simulator
