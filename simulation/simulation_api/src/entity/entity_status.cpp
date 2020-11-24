// Copyright 2015-2020 TierIV.inc. All rights reserved.
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

#include <simulation_api/entity/entity_status.hpp>

namespace simulation_api
{
namespace entity
{
EntityStatus::EntityStatus(
  double t,
  geometry_msgs::msg::Pose pose,
  geometry_msgs::msg::Twist twist,
  geometry_msgs::msg::Accel accel)
: time(t),
  twist(twist),
  accel(accel),
  pose(pose)
{
  coordinate = WORLD;
}

EntityStatus::EntityStatus(
  double t,
  std::int64_t lanelet_id, double s, double offset,
  geometry_msgs::msg::Vector3 rpy,
  geometry_msgs::msg::Twist twist,
  geometry_msgs::msg::Accel accel)
: time(t),
  twist(twist),
  accel(accel),
  lanelet_id(lanelet_id),
  offset(offset),
  s(s),
  rpy(rpy)
{
  coordinate = LANE;
}

openscenario_msgs::msg::EntityStatus EntityStatus::toRosMsg() const
{
  openscenario_msgs::msg::EntityStatus ret;
  ret.time = time;
  if (coordinate == CoordinateFrameTypes::WORLD) {
    ret.coordinate = ret.WORLD;
  }
  if (coordinate == CoordinateFrameTypes::LANE) {
    ret.coordinate = ret.LANE;
  }
  ret.twist = twist;
  ret.accel = accel;
  ret.lanelet_id = lanelet_id;
  ret.offset = offset;
  ret.s = s;
  ret.rpy = rpy;
  ret.pose = pose;
  return ret;
}
}      // namespace entity
}  // namespace simulation_api
