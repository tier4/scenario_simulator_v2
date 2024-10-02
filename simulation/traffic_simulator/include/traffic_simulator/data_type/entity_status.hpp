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

#ifndef TRAFFIC_SIMULATOR__DATA_TYPE__ENTITY_STATUS_HPP_
#define TRAFFIC_SIMULATOR__DATA_TYPE__ENTITY_STATUS_HPP_

#include <traffic_simulator/data_type/lanelet_pose.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>

namespace traffic_simulator
{
using EntityStatus = traffic_simulator_msgs::msg::EntityStatus;
using EntityType = traffic_simulator_msgs::msg::EntityType;
using EntitySubtype = traffic_simulator_msgs::msg::EntitySubtype;

inline namespace entity_status
{
class CanonicalizedEntityStatus
{
public:
  explicit CanonicalizedEntityStatus(
    const EntityStatus & may_non_canonicalized_entity_status,
    const std::optional<CanonicalizedLaneletPose> & canonicalized_lanelet_pose);
  explicit CanonicalizedEntityStatus(const CanonicalizedEntityStatus & obj);
  explicit operator EntityStatus() const noexcept { return entity_status_; }

  auto set(const CanonicalizedEntityStatus & status) -> void;
  auto set(
    const EntityStatus & status, const lanelet::Ids & lanelet_ids, const double matching_distance,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> void;
  auto set(
    const EntityStatus & status, const double matching_distance,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> void;

  auto setAction(const std::string & action) -> void;
  auto getActionStatus() const noexcept -> const traffic_simulator_msgs::msg::ActionStatus &;

  auto getTime() const noexcept -> double;
  auto setTime(double) -> void;

  auto getMapPose() const noexcept -> const geometry_msgs::msg::Pose &;
  auto setMapPose(const geometry_msgs::msg::Pose & pose) -> void;

  auto getTwist() const noexcept -> const geometry_msgs::msg::Twist &;
  auto setTwist(const geometry_msgs::msg::Twist & twist) -> void;
  auto setLinearVelocity(double linear_velocity) -> void;

  auto getAccel() const noexcept -> const geometry_msgs::msg::Accel &;
  auto setAccel(const geometry_msgs::msg::Accel & accel) -> void;
  auto setLinearAcceleration(double linear_acceleration) -> void;

  auto getLinearJerk() const noexcept -> double;
  auto setLinearJerk(double) -> void;

  auto laneMatchingSucceed() const noexcept -> bool;
  auto getLaneletId() const noexcept -> lanelet::Id;
  auto getLaneletIds() const noexcept -> lanelet::Ids;
  auto getLaneletPose() const noexcept -> const LaneletPose &;
  auto getCanonicalizedLaneletPose() const noexcept
    -> const std::optional<CanonicalizedLaneletPose> &;
  auto getName() const noexcept -> const std::string & { return entity_status_.name; }
  auto getType() const noexcept -> const EntityType & { return entity_status_.type; }
  auto getSubtype() const noexcept -> const EntitySubtype & { return entity_status_.subtype; }
  auto getBoundingBox() const noexcept -> const traffic_simulator_msgs::msg::BoundingBox &;

private:
  std::optional<CanonicalizedLaneletPose> canonicalized_lanelet_pose_;
  EntityStatus entity_status_;
};
}  // namespace entity_status
auto isSameLaneletId(
  const CanonicalizedEntityStatus & first_status, const CanonicalizedEntityStatus & second_status)
  -> bool;
auto isSameLaneletId(const CanonicalizedEntityStatus & status, const lanelet::Id lanelet_id)
  -> bool;
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__ENTITY_STATUS_HPP_
