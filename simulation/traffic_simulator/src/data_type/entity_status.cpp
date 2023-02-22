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
CanonicalizedEntityStatusType::CanonicalizedEntityStatusType(
  const traffic_simulator_msgs::msg::EntityStatus & may_non_canonicalized_entity_status,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils)
: entity_status_(canonicalize(may_non_canonicalized_entity_status, hdmap_utils))
{
}

auto CanonicalizedEntityStatusType::canonicalize(
  const traffic_simulator_msgs::msg::EntityStatus & may_non_canonicalized_entity_status,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils)
  -> traffic_simulator_msgs::msg::EntityStatus
{
  auto canonicalized = may_non_canonicalized_entity_status;
  if (may_non_canonicalized_entity_status.lanelet_pose_valid) {
    canonicalized.lanelet_pose_valid = true;
    canonicalized.lanelet_pose = static_cast<traffic_simulator_msgs::msg::LaneletPose>(
      traffic_simulator::lanelet_pose::CanonicalizedLaneletPose(
        may_non_canonicalized_entity_status.lanelet_pose, hdmap_utils));
  } else {
    canonicalized.lanelet_pose_valid = false;
    canonicalized.lanelet_pose = traffic_simulator_msgs::msg::LaneletPose();
  }
  return canonicalized;
}
}  // namespace entity_status
}  // namespace traffic_simulator
