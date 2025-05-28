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
inline namespace entity_status
{
CanonicalizedEntityStatus::CanonicalizedEntityStatus(
  const EntityStatus & may_non_canonicalized_entity_status,
  const std::vector<std::optional<CanonicalizedLaneletPose>> & canonicalized_lanelet_poses)
: canonicalized_lanelet_poses_{canonicalized_lanelet_poses},
  entity_status_{may_non_canonicalized_entity_status}
{
  for (const auto & canonicalized_lanelet_pose : canonicalized_lanelet_poses_) {
    if (canonicalized_lanelet_pose) {
      entity_status_.lanelet_poses.emplace_back(
        static_cast<LaneletPose>(canonicalized_lanelet_pose.value()));
      /*
        The position in Oz axis and orientation based on LaneletPose are rewritten to
        the used msg::Pose (map_pose) since such adjustment relative to the lanelet is necessary,
        The position in Ox and Oy axis is not rewritten because the map_pose retrieved via
        lanelet_pose = pose::toCanonicalizedLaneletPose(map_pose), then map_pose pose::toMapPose(lanelet_pose)
        can be slightly different from the original one (especially if the entity changes lane).
      */
      const auto map_pose_based_on_lanelet_pose =
        static_cast<geometry_msgs::msg::Pose>(canonicalized_lanelet_pose.value());
      entity_status_.pose.position.z = map_pose_based_on_lanelet_pose.position.z;
      entity_status_.pose.orientation = map_pose_based_on_lanelet_pose.orientation;
    } else {
      entity_status_.lanelet_poses.emplace_back(LaneletPose());
    }
    assert(entity_status_.lanelet_poses.back().lanelet_pose_valid == canonicalized_lanelet_pose.has_value());
  }
}

CanonicalizedEntityStatus::CanonicalizedEntityStatus(const CanonicalizedEntityStatus & obj)
: canonicalized_lanelet_poses_(obj.canonicalized_lanelet_poses_),
  entity_status_(static_cast<EntityStatus>(obj))
{
}

auto CanonicalizedEntityStatus::set(const CanonicalizedEntityStatus & status) -> void
{
  assert(getType() == status.getType());
  assert(getSubtype() == status.getSubtype());
  assert(getName() == status.getName());
  assert(getBoundingBox() == status.getBoundingBox());
  entity_status_ = status.entity_status_;
  canonicalized_lanelet_poses_ = status.canonicalized_lanelet_poses_;
}

auto CanonicalizedEntityStatus::set(
  const EntityStatus & status, const lanelet::Ids & lanelet_ids, const double matching_distance)
  -> void
{
  const auto include_crosswalk =
    getType().type == traffic_simulator_msgs::msg::EntityType::PEDESTRIAN ||
    getType().type == traffic_simulator_msgs::msg::EntityType::MISC_OBJECT;

  std::vector<std::optional<CanonicalizedLaneletPose>> canonicalized_lanelet_poses;
  // if (status.lanelet_pose_valid) {
  //   canonicalized_lanelet_pose = pose::toCanonicalizedLaneletPose(status.lanelet_poses);
  // } else {
  //   // prefer the current lanelet
  //   canonicalized_lanelet_pose = pose::toCanonicalizedLaneletPose(
  //     status.pose, getBoundingBox(), lanelet_ids, include_crosswalk, matching_distance);
  // }
  for (const auto & lanelet_pose : status.lanelet_poses) {
    if (lanelet_pose.lanelet_pose_valid) {
      canonicalized_lanelet_poses.emplace_back(
        pose::toCanonicalizedLaneletPose(lanelet_pose));
    } else {
      canonicalized_lanelet_poses.emplace_back(
        pose::toCanonicalizedLaneletPose(
          status.pose, getBoundingBox(), lanelet_ids, include_crosswalk, matching_distance));
    }
  }
  set(CanonicalizedEntityStatus(status, canonicalized_lanelet_poses));
}

auto CanonicalizedEntityStatus::set(const EntityStatus & status, const double matching_distance)
  -> void
{
  set(status, getLaneletIds(), matching_distance);
}

auto CanonicalizedEntityStatus::setAction(const std::string & action) -> void
{
  entity_status_.action_status.current_action = action;
}

auto CanonicalizedEntityStatus::getActionStatus() const noexcept
  -> const traffic_simulator_msgs::msg::ActionStatus &
{
  return entity_status_.action_status;
}

auto CanonicalizedEntityStatus::isInLanelet() const noexcept -> bool
{
  // return canonicalized_lanelet_pose_.has_value();
  for (const auto & canonicalized_lanelet_pose : canonicalized_lanelet_poses_) {
    if (canonicalized_lanelet_pose.has_value()) {
      return true;
    }
  }
}

auto CanonicalizedEntityStatus::getBoundingBox() const noexcept
  -> const traffic_simulator_msgs::msg::BoundingBox &
{
  return entity_status_.bounding_box;
}

auto CanonicalizedEntityStatus::setMapPose(const geometry_msgs::msg::Pose & pose) -> void
{
  entity_status_.pose = pose;
}

auto CanonicalizedEntityStatus::getMapPose() const noexcept -> const geometry_msgs::msg::Pose &
{
  return entity_status_.pose;
}

auto CanonicalizedEntityStatus::getAltitude() const -> double
{
  // return canonicalized_lanelet_pose_ ? canonicalized_lanelet_pose_->getAltitude()
  //                                    : entity_status_.pose.position.z;
  if (canonicalized_lanelet_poses_.empty()) {
    THROW_SEMANTIC_ERROR("Target entity status did not matched to lanelet pose.");
  }
}

auto CanonicalizedEntityStatus::getLaneletPoses() const -> const std::vector<LaneletPose> &
{
  // if (canonicalized_lanelet_pose_) {
  //   return canonicalized_lanelet_pose_->getLaneletPose();
  // } else {
  //   THROW_SEMANTIC_ERROR("Target entity status did not matched to lanelet pose.");
  // }
  if (canonicalized_lanelet_poses_.empty()) {
    THROW_SEMANTIC_ERROR("Target entity status did not matched to lanelet pose.");
  }
  return entity_status_.lanelet_poses;
}

auto CanonicalizedEntityStatus::getLaneletId() const -> lanelet::Id
{
  // return getLaneletPose().lanelet_id;
  // ###########################################################
  // WIP this part is to heavy to refactor, so just return the first one
  // ###########################################################
  if (canonicalized_lanelet_poses_.empty()) {
    THROW_SEMANTIC_ERROR("Target entity status did not matched to lanelet pose.");
  }
  return canonicalized_lanelet_poses_.front()->getLaneletId();
}

auto CanonicalizedEntityStatus::getLaneletIds() const -> lanelet::Ids
{
  return isInLanelet() ? lanelet::Ids{getLaneletId()} : lanelet::Ids{};
}

auto CanonicalizedEntityStatus::getCanonicalizedLaneletPose() const noexcept
  -> const std::optional<CanonicalizedLaneletPose> &
{
  // return canonicalized_lanelet_pose_;
  // ###########################################################
  // WIP this part is to heavy to refactor, so just return the first one
  // ###########################################################
  if (canonicalized_lanelet_poses_.empty()) {
    THROW_SEMANTIC_ERROR("Target entity status did not matched to lanelet pose.");
  }
  return canonicalized_lanelet_poses_.front();
}

auto CanonicalizedEntityStatus::setTwist(const geometry_msgs::msg::Twist & twist) -> void
{
  entity_status_.action_status.twist = twist;
}

auto CanonicalizedEntityStatus::getTwist() const noexcept -> const geometry_msgs::msg::Twist &
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

auto CanonicalizedEntityStatus::getAccel() const noexcept -> const geometry_msgs::msg::Accel &
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

auto isSameLaneletId(
  const CanonicalizedEntityStatus & first_status, const CanonicalizedEntityStatus & second_status)
  -> bool
{
  return first_status.getLaneletId() == second_status.getLaneletId();
}

auto isSameLaneletId(const CanonicalizedEntityStatus & status, const lanelet::Id lanelet_id) -> bool
{
  return status.getLaneletId() == lanelet_id;
}
}  // namespace traffic_simulator
