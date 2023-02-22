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
namespace entity_status
{
class CanonicalizedEntityStatusType
{
public:
  explicit CanonicalizedEntityStatusType(
    const traffic_simulator_msgs::msg::EntityStatus & may_non_canonicalized_entity_status,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils);
  explicit operator traffic_simulator_msgs::msg::EntityStatus() const noexcept
  {
    return entity_status_;
  }

private:
  auto canonicalize(
    const traffic_simulator_msgs::msg::EntityStatus & may_non_canonicalized_entity_status,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils)
    -> traffic_simulator_msgs::msg::EntityStatus;
  const traffic_simulator_msgs::msg::EntityStatus entity_status_;
};
}  // namespace entity_status
}  // namespace traffic_simulator

using EntityStatusType = traffic_simulator_msgs::msg::EntityStatus;
using CanonicalizedEntityStatusType =
  traffic_simulator::entity_status::CanonicalizedEntityStatusType;

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__ENTITY_STATUS_HPP_
