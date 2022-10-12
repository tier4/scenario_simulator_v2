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

#include <traffic_simulator/entity/misc_object_entity.hpp>

namespace traffic_simulator
{
namespace entity
{
auto MiscObjectEntity::getEntityType() const -> const traffic_simulator_msgs::msg::EntityType &
{
  static const auto entity_type = []() {
    traffic_simulator_msgs::msg::EntityType entity_type;
    entity_type.type = traffic_simulator_msgs::msg::EntityType::MISC_OBJECT;
    return entity_type;
  }();

  return entity_type;
}

void MiscObjectEntity::onUpdate(double, double)
{
  status_.action_status.accel = geometry_msgs::msg::Accel();
  status_.action_status.twist = geometry_msgs::msg::Twist();
  status_.action_status.current_action = "static";
  status_before_update_ = status_;
}

auto MiscObjectEntity::getCurrentAction() const -> std::string
{
  if (not npc_logic_started_) {
    return "waiting";
  } else {
    return status_.action_status.current_action;
  }
}

auto MiscObjectEntity::getDriverModel() const -> traffic_simulator_msgs::msg::DriverModel
{
  THROW_SEMANTIC_ERROR("getDriverModel function does not support in MiscObjectEntity.");
}

void MiscObjectEntity::setDriverModel(const traffic_simulator_msgs::msg::DriverModel &)
{
  THROW_SEMANTIC_ERROR("setDriverModel function does not support in MiscObjectEntity.");
}
}  // namespace entity
}  // namespace traffic_simulator
