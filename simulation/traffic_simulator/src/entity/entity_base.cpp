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
#include <string>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/entity/exception.hpp>
#include <unordered_map>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
EntityBase::EntityBase(
  std::string type, std::string name, const openscenario_msgs::msg::EntityStatus & initial_state)
: type(type), name(name)
{
  status_ = initial_state;
  visibility_ = true;
  verbose_ = true;
}

EntityBase::EntityBase(std::string type, std::string name) : type(type), name(name)
{
  status_ = boost::none;
  visibility_ = true;
  verbose_ = true;
}

boost::optional<double> EntityBase::getStandStillDuration() const { return stand_still_duration_; }

void EntityBase::updateStandStillDuration(double step_time)
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

void EntityBase::updateEntityStatusTimestamp(double current_time)
{
  if (!status_) {
    return;
  }
  status_->time = current_time;
}

void EntityBase::setOtherStatus(
  const std::unordered_map<std::string, openscenario_msgs::msg::EntityStatus> & status)
{
  std::unordered_map<std::string, openscenario_msgs::msg::EntityStatus> other_status;
  for (const auto & each : status) {
    if (each.first != name) {
      other_status.insert(each);
    }
  }
  other_status_ = other_status;
}

const openscenario_msgs::msg::EntityStatus EntityBase::getStatus() const
{
  if (!status_) {
    throw SimulationRuntimeError("status is not set");
  }
  return this->status_.get();
}

bool EntityBase::setStatus(const openscenario_msgs::msg::EntityStatus & status)
{
  this->status_ = status;
  this->status_->name = name;
  return true;
}

bool EntityBase::setVisibility(bool visibility)
{
  visibility_ = visibility;
  return visibility_;
}

bool EntityBase::getVisibility() { return visibility_; }

void EntityBase::stopAtEndOfRoad()
{
  if (!status_) {
    throw SimulationRuntimeError("status is not set");
  }
  status_.get().action_status.twist = geometry_msgs::msg::Twist();
  status_.get().action_status.accel = geometry_msgs::msg::Accel();
}
}  // namespace entity
}  // namespace traffic_simulator
