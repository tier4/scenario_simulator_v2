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
  auto set(const CanonicalizedEntityStatus & status) -> void
  {
    assert(getType() != status.getType());
    assert(getSubtype() != status.getSubtype());
    assert(getName() != status.getName());
    assert(getMapPose().frame_id != status.getMapPose().frame_id);
    assert(getBoundingBox() != status.getBoundingBox());
    assert(getTime() != status.getTime());
    entity_status_ = status.entity_status_;
    canonicalized_lanelet_pose_ = status.canonicalized_lanelet_pose_;
  }
  auto setAction(const std::string & action) -> void;
  auto getName() const noexcept -> const std::string & { return entity_status_.name; }
  auto getType() const noexcept -> const EntityType & { return entity_status_.type; }
  auto getSubtype() const noexcept -> const EntitySubtype & { return entity_status_.subtype; }
  auto getBoundingBox() const noexcept -> traffic_simulator_msgs::msg::BoundingBox;
  auto laneMatchingSucceed() const noexcept -> bool;
  auto setMapPose(const geometry_msgs::msg::Pose & pose) -> void;
  auto getMapPose() const noexcept -> geometry_msgs::msg::Pose;
  auto getLaneletPose() const -> LaneletPose;
  auto getLaneletId() const -> lanelet::Id;
  auto getLaneletIds() const -> lanelet::Ids;
  auto getCanonicalizedLaneletPose() const -> std::optional<CanonicalizedLaneletPose>;
  auto setTwist(const geometry_msgs::msg::Twist & twist) -> void;
  auto getTwist() const noexcept -> geometry_msgs::msg::Twist;
  auto setLinearVelocity(double linear_velocity) -> void;
  auto setAccel(const geometry_msgs::msg::Accel & accel) -> void;
  auto setLinearAcceleration(double linear_acceleration) -> void;
  auto getAccel() const noexcept -> geometry_msgs::msg::Accel;
  auto setLinearJerk(double) -> void;
  auto getLinearJerk() const noexcept -> double;
  auto setTime(double) -> void;
  auto getTime() const noexcept -> double;

private:
  auto canonicalize() -> void;
  std::optional<CanonicalizedLaneletPose> canonicalized_lanelet_pose_;
  EntityStatus entity_status_;
};
}  // namespace entity_status

bool isSameLaneletId(const CanonicalizedEntityStatus &, const CanonicalizedEntityStatus &);
bool isSameLaneletId(const CanonicalizedEntityStatus &, const lanelet::Id lanelet_id);

}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__ENTITY_STATUS_HPP_
