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
  const std::string & type, const std::string & name,
  const openscenario_msgs::msg::EntityStatus & status)
: type(type), name(name), status_(status), verbose_(true), visibility_(true)
{
}

void EntityBase::onUpdate(double, double) { status_before_update_ = status_; }

double EntityBase::getStandStillDuration() const { return stand_still_duration_; }

const autoware_vehicle_msgs::msg::VehicleCommand EntityBase::getVehicleCommand()
{
  THROW_SIMULATION_ERROR("get vehicle command does not support in ", type, " entity type");
}

void EntityBase::updateStandStillDuration(const double step_time)
{
  if (std::fabs(status_.action_status.twist.linear.x) <= std::numeric_limits<double>::epsilon()) {
    stand_still_duration_ = step_time + stand_still_duration_;
  } else {
    stand_still_duration_ = 0;
  }
}

void EntityBase::updateEntityStatusTimestamp(const double current_time)
{
  status_.time = current_time;
}

void EntityBase::setOtherStatus(
  const std::unordered_map<std::string, openscenario_msgs::msg::EntityStatus> & status)
{
  other_status_.clear();
  for (const auto & each : status) {
    if (each.first != name) {
      const auto p0 = each.second.pose.position;
      const auto p1 = status_.pose.position;
      double distance =
        std::sqrt(std::pow(p0.x - p1.x, 2) + std::pow(p0.y - p1.y, 2) + std::pow(p0.z - p1.z, 2));
      if (distance < 30) {
        other_status_.insert(each);
      }
    }
  }
}

const openscenario_msgs::msg::EntityStatus EntityBase::getStatus() const { return this->status_; }

void EntityBase::setStatus(const openscenario_msgs::msg::EntityStatus & status)
{
  status_ = status;
  status_.name = name;
}

void EntityBase::stopAtEndOfRoad()
{
  status_.action_status.twist = geometry_msgs::msg::Twist();
  status_.action_status.accel = geometry_msgs::msg::Accel();
}
}  // namespace entity
}  // namespace traffic_simulator
