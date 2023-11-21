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

#include <limits>
#include <traffic_simulator/entity/deleted_entity.hpp>

namespace traffic_simulator
{
namespace entity
{
DeletedEntity::DeletedEntity(
  const std::string & name, const CanonicalizedEntityStatus & entity_status,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
: EntityBase(name, entity_status, hdmap_utils_ptr)
{
  auto status = static_cast<EntityStatus>(status_);
  status.action_status.twist = geometry_msgs::msg::Twist();
  status.action_status.accel = geometry_msgs::msg::Accel();
  status.action_status.linear_jerk = 0;
  status.action_status.current_action = "static";
  status.bounding_box.dimensions.x = 0;
  status.bounding_box.dimensions.y = 0;
  status.bounding_box.dimensions.z = 0;
  status.pose.position.x = std::numeric_limits<double>::infinity();
  status.pose.position.y = std::numeric_limits<double>::infinity();
  status.pose.position.z = std::numeric_limits<double>::infinity();
  status.lanelet_pose_valid = false;
  status_ = CanonicalizedEntityStatus(status, hdmap_utils_ptr_);
  status_before_update_ = CanonicalizedEntityStatus(status, hdmap_utils_ptr_);
}

auto DeletedEntity::getCurrentAction() const -> std::string { return "static"; }

auto DeletedEntity::getBehaviorParameter() const -> traffic_simulator_msgs::msg::BehaviorParameter
{
  return traffic_simulator_msgs::msg::BehaviorParameter();
}

auto DeletedEntity::getDefaultDynamicConstraints() const
  -> const traffic_simulator_msgs::msg::DynamicConstraints &
{
  static const auto default_dynamic_constraints = []() {
    auto dynamic_constraints = traffic_simulator_msgs::msg::DynamicConstraints();
    dynamic_constraints.max_speed = 0.0;
    dynamic_constraints.max_acceleration = 0.0;
    dynamic_constraints.max_acceleration_rate = 0.0;
    dynamic_constraints.max_deceleration = 0.0;
    dynamic_constraints.max_deceleration_rate = 0.0;
    return dynamic_constraints;
  }();

  return default_dynamic_constraints;
}

auto DeletedEntity::fillLaneletPose(CanonicalizedEntityStatus &) -> void {}

}  // namespace entity
}  // namespace traffic_simulator
