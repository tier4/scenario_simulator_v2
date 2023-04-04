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
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

namespace traffic_simulator
{
using EntityStatusType = traffic_simulator_msgs::msg::EntityStatus;

namespace entity_status
{
class CanonicalizedEntityStatusType
{
public:
  explicit CanonicalizedEntityStatusType(
    const traffic_simulator_msgs::msg::EntityStatus & may_non_canonicalized_entity_status,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils);
  explicit CanonicalizedEntityStatusType(
    const traffic_simulator_msgs::msg::EntityStatus & may_non_canonicalized_entity_status,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils,
    const std::vector<std::int64_t> & route_lanelets);
  explicit CanonicalizedEntityStatusType(const CanonicalizedEntityStatusType & obj);
  explicit operator traffic_simulator_msgs::msg::EntityStatus() const noexcept
  {
    return entity_status_;
  }
  explicit operator geometry_msgs::msg::Pose() const noexcept { return entity_status_.pose; }
  explicit operator traffic_simulator_msgs::msg::LaneletPose() const
  {
    if (!laneMatchingSucceed()) {
      THROW_SEMANTIC_ERROR("Target entity status did not matched to lanelet pose.");
    }
    return entity_status_.lanelet_pose;
  }
  CanonicalizedEntityStatusType & operator=(const CanonicalizedEntityStatusType & obj)
  {
    this->entity_status_ = obj.entity_status_;
    return *this;
  }
  bool laneMatchingSucceed() const noexcept { return entity_status_.lanelet_pose_valid; }
  void setTwist(const geometry_msgs::msg::Twist & twist = geometry_msgs::msg::Twist());
  void setLinearVelocity(double linear_velocity);
  void setAccel(const geometry_msgs::msg::Accel & accel = geometry_msgs::msg::Accel());
  void setTime(double time);

private:
  auto canonicalize(
    const EntityStatusType & may_non_canonicalized_entity_status,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils) -> EntityStatusType;
  auto canonicalize(
    const EntityStatusType & may_non_canonicalized_entity_status,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils,
    const std::vector<std::int64_t> & route_lanelets) -> EntityStatusType;
  EntityStatusType entity_status_;
};
}  // namespace entity_status

using CanonicalizedEntityStatusType = entity_status::CanonicalizedEntityStatusType;

}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__ENTITY_STATUS_HPP_
