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

#include <do_nothing_plugin/plugin.hpp>

namespace entity_behavior
{
void DoNothingBehavior::configure(const rclcpp::Logger &) {}

void DoNothingBehavior::update(double current_time, double)
{
  if (getRequest() == traffic_simulator::behavior::Request::FOLLOW_POLYLINE_TRAJECTORY) {
    followPolylineTrajectory(current_time);
  }
  entity_status_->setTime(current_time);
  setUpdatedStatus(entity_status_);
}

void DoNothingBehavior::checkPolylineTrajectory()
{
  if (const auto trajectory = getPolylineTrajectory()) {
    if (trajectory->closed) {
      THROW_SIMULATION_ERROR("Currentry, closed trajectory does not supported.");
    }
    if (trajectory->dynamic_constraints_ignorable) {
      THROW_SIMULATION_ERROR(
        "Currentry, dynamic_constraints_ignorable = true (in OpenSCENARIO, followingMode = "
        "follow) does not support in DoNothingBehavior.");
    }
    if (std::abs(trajectory->initial_distance_offset) <= std::numeric_limits<double>::epsilon()) {
      THROW_SIMULATION_ERROR(
        "Currentry, initial_distance_offset should be 0 when following trajectory in "
        "DoNothingBehavior.");
    }
    if (std::abs(trajectory->base_time) <= std::numeric_limits<double>::epsilon()) {
      THROW_SIMULATION_ERROR(
        "Currentry, base_time should be 0 when following trajectory in "
        "DoNothingBehavior.");
    }
  } else {
    THROW_SIMULATION_ERROR(
      "Traffic simulator send requests of FollowTrajectory, but the trajectory is empty.",
      "This message is not originally intended to be displayed, if you see it, please "
      "contact the developer of traffic_simulator.");
  }
}

void DoNothingBehavior::followPolylineTrajectory(double current_time)
{
  checkPolylineTrajectory();
  if (const auto trajectory = getPolylineTrajectory()) {
  }
}

const std::string & DoNothingBehavior::getCurrentAction() const
{
  static const std::string behavior = "do_nothing";
  return behavior;
}
}  // namespace entity_behavior

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(entity_behavior::DoNothingBehavior, entity_behavior::BehaviorPluginBase)
