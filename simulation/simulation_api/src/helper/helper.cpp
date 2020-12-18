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

#include <simulation_api/helper/helper.hpp>

namespace simulation_api
{
namespace helper
{
openscenario_msgs::msg::ActionStatus constractActionStatus(
  double linear_vel = 0,
  double angular_vel = 0,
  double linear_accel = 0,
  double angular_accel = 0)
{
  openscenario_msgs::msg::ActionStatus status;
  geometry_msgs::msg::Twist twist;
  twist.linear.x = linear_vel;
  twist.angular.z = angular_vel;
  geometry_msgs::msg::Accel accel;
  accel.linear.x = linear_accel;
  accel.angular.z = angular_accel;
  status.twist = twist;
  status.accel = accel;
  return status;
}
}  // namespace helper
}  // namespace simulation_api
