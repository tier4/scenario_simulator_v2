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

namespace entity_status
{
class CanonicalizedEntityStatus
{
public:
  explicit CanonicalizedEntityStatus(
    const traffic_simulator_msgs::msg::EntityStatus & may_non_canonicalized_entity_status,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils);
  explicit CanonicalizedEntityStatus(
    const traffic_simulator_msgs::msg::EntityStatus & may_non_canonicalized_entity_status,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils,
    const std::vector<std::int64_t> & route_lanelets);
  explicit CanonicalizedEntityStatus(const CanonicalizedEntityStatus & obj);
  explicit operator traffic_simulator_msgs::msg::EntityStatus() const noexcept
  {
    return entity_status_;
  }
  CanonicalizedEntityStatus & operator=(const CanonicalizedEntityStatus & obj)
  {
    this->entity_status_ = obj.entity_status_;
    return *this;
  }
  auto laneMatchingSucceed() const noexcept -> bool { return entity_status_.lanelet_pose_valid; }
  auto getMapPose() const noexcept -> geometry_msgs::msg::Pose { return entity_status_.pose; }
  auto getLaneletPose() const -> LaneletPose;
  auto setTwist(const geometry_msgs::msg::Twist & twist) -> void;
  auto getTwist() const noexcept -> geometry_msgs::msg::Twist;
  auto setLinearVelocity(double linear_velocity) -> void;
  auto setAccel(const geometry_msgs::msg::Accel & accel) -> void;
  auto getAccel() -> geometry_msgs::msg::Accel;
  auto setLinearJerk(double) -> void;
  auto getLinearJerk() -> double;

private:
  auto canonicalize(
    const EntityStatus & may_non_canonicalized_entity_status,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils) -> EntityStatus;
  auto canonicalize(
    const EntityStatus & may_non_canonicalized_entity_status,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils,
    const std::vector<std::int64_t> & route_lanelets) -> EntityStatus;
  EntityStatus entity_status_;
};
}  // namespace entity_status

using CanonicalizedEntityStatus = entity_status::CanonicalizedEntityStatus;

}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__ENTITY_STATUS_HPP_
