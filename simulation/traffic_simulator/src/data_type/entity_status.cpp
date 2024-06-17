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
  const std::optional<CanonicalizedLaneletPose> & canonicalized_lanelet_pose)
: canonicalized_lanelet_pose_{canonicalized_lanelet_pose},
  entity_status_{may_non_canonicalized_entity_status}
{
  canonicalize();
}

/// @todo this constructor will be removed (after adaptation of behavior_tree)
CanonicalizedEntityStatus::CanonicalizedEntityStatus(
  const EntityStatus & may_non_canonicalized_entity_status,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils)
: canonicalized_lanelet_pose_{may_non_canonicalized_entity_status.lanelet_pose_valid?std::optional(CanonicalizedLaneletPose(
    may_non_canonicalized_entity_status.lanelet_pose, hdmap_utils)):std::nullopt},
  entity_status_{may_non_canonicalized_entity_status}
{
  canonicalize();
}

/// @todo this constructor will be removed (after adaptation of behavior_tree)
CanonicalizedEntityStatus::CanonicalizedEntityStatus(
  const EntityStatus & may_non_canonicalized_entity_status,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils, const lanelet::Ids & route_lanelets)
: canonicalized_lanelet_pose_{may_non_canonicalized_entity_status.lanelet_pose_valid?std::optional(CanonicalizedLaneletPose(
    may_non_canonicalized_entity_status.lanelet_pose, route_lanelets, hdmap_utils )):std::nullopt},
      entity_status_{may_non_canonicalized_entity_status}
{
  canonicalize();
}

CanonicalizedEntityStatus::CanonicalizedEntityStatus(const CanonicalizedEntityStatus & obj)
: canonicalized_lanelet_pose_(obj.canonicalized_lanelet_pose_),
  entity_status_(static_cast<EntityStatus>(obj))
{
}

CanonicalizedEntityStatus & CanonicalizedEntityStatus::operator=(
  const CanonicalizedEntityStatus & obj)
{
  this->canonicalized_lanelet_pose_ = obj.canonicalized_lanelet_pose_;
  this->entity_status_ = obj.entity_status_;
  return *this;
}

auto CanonicalizedEntityStatus::canonicalize() -> void
{
  assert(entity_status_.lanelet_pose_valid == canonicalized_lanelet_pose_.has_value());
  if (canonicalized_lanelet_pose_) {
    entity_status_.lanelet_pose_valid = true;
    entity_status_.lanelet_pose = static_cast<LaneletPose>(canonicalized_lanelet_pose_.value());
    /*
      The position in Oz axis and orientation based on LaneletPose are rewritten to
      the used msg::Pose (map_pose) since such adjustment relative to the lanelet is necessary,
      The position in Ox and Oy axis is not rewritten because the map_pose retrieved via
      lanelet_pose = toCanonicalizedLaneletPose(map_pose), then map_pose = toMapPose(lanelet_pose)
      can be slightly different from the original one (especially if the entity changes lane).
    */
    const auto map_pose_based_on_lanelet_pose =
      static_cast<geometry_msgs::msg::Pose>(canonicalized_lanelet_pose_.value());
    entity_status_.pose.position.z = map_pose_based_on_lanelet_pose.position.z;
    entity_status_.pose.orientation = map_pose_based_on_lanelet_pose.orientation;
  } else {
    entity_status_.lanelet_pose_valid = false;
    entity_status_.lanelet_pose = LaneletPose();
  }
}

auto CanonicalizedEntityStatus::laneMatchingSucceed() const noexcept -> bool
{
  return canonicalized_lanelet_pose_.has_value();
}

auto CanonicalizedEntityStatus::getBoundingBox() const noexcept
  -> traffic_simulator_msgs::msg::BoundingBox
{
  return entity_status_.bounding_box;
}

auto CanonicalizedEntityStatus::getMapPose() const noexcept -> geometry_msgs::msg::Pose
{
  return entity_status_.pose;
}

auto CanonicalizedEntityStatus::getLaneletPose() const -> LaneletPose
{
  if (canonicalized_lanelet_pose_) {
    return static_cast<LaneletPose>(canonicalized_lanelet_pose_.value());
  } else {
    THROW_SEMANTIC_ERROR("Target entity status did not matched to lanelet pose.");
  }
}

auto CanonicalizedEntityStatus::getLaneletId() const -> lanelet::Id
{
  return getLaneletPose().lanelet_id;
}

auto CanonicalizedEntityStatus::getLaneletIds() const -> lanelet::Ids
{
  return laneMatchingSucceed() ? lanelet::Ids{getLaneletId()} : lanelet::Ids{};
}

auto CanonicalizedEntityStatus::getCanonicalizedLaneletPose() const
  -> std::optional<CanonicalizedLaneletPose>
{
  return canonicalized_lanelet_pose_;
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

auto CanonicalizedEntityStatus::setLinearAcceleration(double linear_acceleration) -> void
{
  entity_status_.action_status.accel.linear.x = linear_acceleration;
}

auto CanonicalizedEntityStatus::getAccel() const noexcept -> geometry_msgs::msg::Accel
{
  return entity_status_.action_status.accel;
}

auto CanonicalizedEntityStatus::setLinearJerk(double linear_jerk) -> void
{
  entity_status_.action_status.linear_jerk = linear_jerk;
}

auto CanonicalizedEntityStatus::setAction(const std::string & action) -> void
{
  entity_status_.action_status.current_action = action;
}

auto CanonicalizedEntityStatus::getLinearJerk() const noexcept -> double
{
  return entity_status_.action_status.linear_jerk;
}

auto CanonicalizedEntityStatus::setTime(double time) -> void { entity_status_.time = time; }

auto CanonicalizedEntityStatus::getTime() const noexcept -> double { return entity_status_.time; }
}  // namespace entity_status

auto isSameLaneletId(const CanonicalizedEntityStatus & s0, const CanonicalizedEntityStatus & s1)
  -> bool
{
  if (const auto s0_canonicalized_lanelet_pose = s0.getCanonicalizedLaneletPose()) {
    if (const auto s1_canonicalized_lanelet_pose = s1.getCanonicalizedLaneletPose()) {
      return isSameLaneletId(
        s0_canonicalized_lanelet_pose.value(), s1_canonicalized_lanelet_pose.value());
    }
  }
  THROW_SIMULATION_ERROR("There is no Lanelet pose");
}

auto isSameLaneletId(const CanonicalizedEntityStatus & s, const lanelet::Id lanelet_id) -> bool
{
  if (const auto s_canonicalized_lanelet_pose = s.getCanonicalizedLaneletPose()) {
    return isSameLaneletId(s_canonicalized_lanelet_pose.value(), lanelet_id);
  }
  THROW_SIMULATION_ERROR("There is no Lanelet pose");
}
}  // namespace traffic_simulator
