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
  const EntityStatus & may_non_canonicalized_entity_status,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils)
: entity_status_(canonicalize(may_non_canonicalized_entity_status, hdmap_utils))
{
}

CanonicalizedEntityStatus::CanonicalizedEntityStatus(
  const EntityStatus & may_non_canonicalized_entity_status,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils, const lanelet::Ids & route_lanelets)
: entity_status_(canonicalize(may_non_canonicalized_entity_status, hdmap_utils, route_lanelets))
{
}

CanonicalizedEntityStatus::CanonicalizedEntityStatus(const CanonicalizedEntityStatus & obj)
: entity_status_(static_cast<EntityStatus>(obj))
{
}

auto CanonicalizedEntityStatus::canonicalize(
  const EntityStatus & may_non_canonicalized_entity_status,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils) -> EntityStatus
{
  auto canonicalized = may_non_canonicalized_entity_status;
  if (may_non_canonicalized_entity_status.lanelet_pose_valid) {
    canonicalized.lanelet_pose_valid = true;
    canonicalized.lanelet_pose = static_cast<LaneletPose>(
      CanonicalizedLaneletPose(may_non_canonicalized_entity_status.lanelet_pose, hdmap_utils));
  } else {
    canonicalized.lanelet_pose_valid = false;
    canonicalized.lanelet_pose = LaneletPose();
  }
  return canonicalized;
}

auto CanonicalizedEntityStatus::canonicalize(
  const EntityStatus & may_non_canonicalized_entity_status,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils, const lanelet::Ids & route_lanelets)
  -> EntityStatus
{
  auto canonicalized = may_non_canonicalized_entity_status;
  if (may_non_canonicalized_entity_status.lanelet_pose_valid) {
    canonicalized.lanelet_pose_valid = true;
    canonicalized.lanelet_pose = static_cast<LaneletPose>(CanonicalizedLaneletPose(
      may_non_canonicalized_entity_status.lanelet_pose, hdmap_utils, route_lanelets));
  } else {
    canonicalized.lanelet_pose_valid = false;
    canonicalized.lanelet_pose = LaneletPose();
  }
  return canonicalized;
}

auto CanonicalizedEntityStatus::getBoundingBox() const noexcept
  -> traffic_simulator_msgs::msg::BoundingBox
{
  return entity_status_.bounding_box;
}

auto CanonicalizedEntityStatus::getLaneletPose() const -> LaneletPose
{
  if (!laneMatchingSucceed()) {
    THROW_SEMANTIC_ERROR("Target entity status did not matched to lanelet pose.");
  }
  return entity_status_.lanelet_pose;
}

auto CanonicalizedEntityStatus::setTwist(const geometry_msgs::msg::Twist & twist) -> void
{
  entity_status_.action_status.twist = twist;
}

auto CanonicalizedEntityStatus::getTwist() const noexcept -> geometry_msgs::msg::Twist
{
  return entity_status_.action_status.twist;
}

auto CanonicalizedEntityStatus::setLinearVelocity(double linear_velocity) -> void
{
  entity_status_.action_status.twist.linear.x = linear_velocity;
}

auto CanonicalizedEntityStatus::setAccel(const geometry_msgs::msg::Accel & accel) -> void
{
  entity_status_.action_status.accel = accel;
}

auto CanonicalizedEntityStatus::getAccel() const noexcept -> geometry_msgs::msg::Accel
{
  return entity_status_.action_status.accel;
}

auto CanonicalizedEntityStatus::setLinearJerk(double linear_jerk) -> void
{
  entity_status_.action_status.linear_jerk = linear_jerk;
}

auto CanonicalizedEntityStatus::getLinearJerk() const noexcept -> double
{
  return entity_status_.action_status.linear_jerk;
}

auto CanonicalizedEntityStatus::setTime(double time) -> void { entity_status_.time = time; }

auto CanonicalizedEntityStatus::getTime() const noexcept -> double { return entity_status_.time; }
}  // namespace entity_status

bool isSameLaneletId(const CanonicalizedEntityStatus & s0, const CanonicalizedEntityStatus & s1)
{
  return s0.getLaneletPose().lanelet_id == s1.getLaneletPose().lanelet_id;
}

bool isSameLaneletId(const CanonicalizedEntityStatus & s, const lanelet::Id lanelet_id)
{
  return s.getLaneletPose().lanelet_id == lanelet_id;
}
}  // namespace traffic_simulator
