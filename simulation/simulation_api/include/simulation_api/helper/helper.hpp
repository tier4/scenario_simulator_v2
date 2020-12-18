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

#ifndef SIMULATION_API__HELPER__HELPER_HPP_
#define SIMULATION_API__HELPER__HELPER_HPP_

#include <openscenario_msgs/msg/lanelet_pose.hpp>
#include <openscenario_msgs/msg/action_status.hpp>

namespace simulation_api
{
namespace helper
{
/**
 * @brief helper function for constracting action status
 *
 * @param linear_vel linear velocity
 * @param angular_vel angluar velocity
 * @param linear_accel linear acceleration
 * @param angular_accel angular acceleration
 * @return openscenario_msgs::msg::ActionStatus
 */
openscenario_msgs::msg::ActionStatus constractActionStatus(
  double linear_vel = 0,
  double angular_vel = 0,
  double linear_accel = 0,
  double angular_accel = 0);

/**
 * @brief helper function for constracting lanelet pose
 *
 * @param lanelet_id lanelet id
 * @param s s value in lane coordinate
 * @param offset offset value in lane coordinate
 * @param roll roll value in the lane coordinate
 * @param pitch pitch value in the lane coordinate
 * @param yaw yaw value in the lane coordinate
 * @return openscenario_msgs::msg::LaneletPose
 */
openscenario_msgs::msg::LaneletPose constractLaneletPose(
  std::int64_t lanelet_id, double s,
  double offset = 0, double roll = 0,
  double pitch = 0, double yaw = 0);
}  // namespace helper
}  // namespace simulation_api

#endif  // SIMULATION_API__HELPER__HELPER_HPP_
