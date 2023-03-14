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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/syntax/dynamic_constraints.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
DynamicConstraints::DynamicConstraints(
  const Double max_acceleration, const Double max_acceleration_rate, const Double max_deceleration,
  const Double max_deceleration_rate, const Double max_speed, const Double max_jerk)
: max_acceleration(max_acceleration),
  max_acceleration_rate(max_acceleration_rate),
  max_deceleration(max_deceleration),
  max_deceleration_rate(max_deceleration_rate),
  max_speed(max_speed),
  max_jerk(max_jerk)
{
}

DynamicConstraints::DynamicConstraints(const pugi::xml_node & node, Scope & scope)
: max_acceleration(readAttribute<Double>("maxAcceleration", node, scope, Double::infinity())),
  max_acceleration_rate(
    readAttribute<Double>("maxAccelerationRate", node, scope, Double::infinity())),
  max_deceleration(readAttribute<Double>("maxDeceleration", node, scope, Double::infinity())),
  max_deceleration_rate(
    readAttribute<Double>("maxDecelerationRate", node, scope, Double::infinity())),
  max_speed(readAttribute<Double>("maxSpeed", node, scope, Double::infinity())),
  max_jerk(readAttribute<Double>("maxJerk", node, scope, Double::infinity()))
{
}

DynamicConstraints::operator traffic_simulator_msgs::msg::DynamicConstraints() const
{
  traffic_simulator_msgs::msg::DynamicConstraints dynamic_constraints;
  dynamic_constraints.max_acceleration = max_acceleration;
  dynamic_constraints.max_acceleration_rate = max_acceleration_rate;
  dynamic_constraints.max_deceleration = max_deceleration;
  dynamic_constraints.max_deceleration_rate = max_deceleration_rate;
  dynamic_constraints.max_speed = max_speed;
  dynamic_constraints.max_jerk = max_jerk;
  return dynamic_constraints;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
