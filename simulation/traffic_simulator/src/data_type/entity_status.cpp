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

#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator/data_type/lanelet_pose.hpp>

namespace traffic_simulator
{
namespace entity_status
{
CanonicalizedEntityStatus::CanonicalizedEntityStatus(
  const EntityStatusType & may_non_canonicalized_entity_status,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils)
: entity_status_(canonicalize(may_non_canonicalized_entity_status, hdmap_utils))
{
}

CanonicalizedEntityStatus::CanonicalizedEntityStatus(
  const traffic_simulator_msgs::msg::EntityStatus & may_non_canonicalized_entity_status,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils,
  const std::vector<std::int64_t> & route_lanelets)
: entity_status_(canonicalize(may_non_canonicalized_entity_status, hdmap_utils, route_lanelets))
{
}

CanonicalizedEntityStatus::CanonicalizedEntityStatus(const CanonicalizedEntityStatus & obj)
: entity_status_(static_cast<EntityStatusType>(obj))
{
}

auto CanonicalizedEntityStatus::canonicalize(
  const EntityStatusType & may_non_canonicalized_entity_status,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils) -> EntityStatusType
{
  auto canonicalized = may_non_canonicalized_entity_status;
  if (may_non_canonicalized_entity_status.lanelet_pose_valid) {
    canonicalized.lanelet_pose_valid = true;
    canonicalized.lanelet_pose = static_cast<LaneletPoseType>(
      CanonicalizedLaneletPose(may_non_canonicalized_entity_status.lanelet_pose, hdmap_utils));
  } else {
    canonicalized.lanelet_pose_valid = false;
    canonicalized.lanelet_pose = LaneletPoseType();
  }
  return canonicalized;
}

auto CanonicalizedEntityStatus::canonicalize(
  const EntityStatusType & may_non_canonicalized_entity_status,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils,
  const std::vector<std::int64_t> & route_lanelets) -> EntityStatusType
{
  auto canonicalized = may_non_canonicalized_entity_status;
  if (may_non_canonicalized_entity_status.lanelet_pose_valid) {
    canonicalized.lanelet_pose_valid = true;
    canonicalized.lanelet_pose = static_cast<LaneletPoseType>(CanonicalizedLaneletPose(
      may_non_canonicalized_entity_status.lanelet_pose, hdmap_utils, route_lanelets));
  } else {
    canonicalized.lanelet_pose_valid = false;
    canonicalized.lanelet_pose = LaneletPoseType();
  }
  return canonicalized;
}

void CanonicalizedEntityStatus::setTwist(const geometry_msgs::msg::Twist & twist)
{
  entity_status_.action_status.twist = twist;
}

void CanonicalizedEntityStatus::setLinearVelocity(double linear_velocity)
{
  entity_status_.action_status.twist.linear.x = linear_velocity;
}

void CanonicalizedEntityStatus::setAccel(const geometry_msgs::msg::Accel & accel)
{
  entity_status_.action_status.accel = accel;
}

void CanonicalizedEntityStatus::setTime(double time) { entity_status_.time = time; }
}  // namespace entity_status
}  // namespace traffic_simulator
