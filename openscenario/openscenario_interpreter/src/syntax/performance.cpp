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
#include <openscenario_interpreter/syntax/performance.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Performance::Performance(
  const Double max_acceleration, const Double max_acceleration_rate, const Double max_deceleration,
  const Double max_deceleration_rate, const Double max_speed)
: max_acceleration(max_acceleration),
  max_acceleration_rate(max_acceleration_rate),
  max_deceleration(max_deceleration),
  max_deceleration_rate(max_deceleration_rate),
  max_speed(max_speed)
{
}

Performance::Performance(const pugi::xml_node & node, Scope & scope)
: max_acceleration(readAttribute<Double>("maxAcceleration", node, scope, Double::infinity())),
  max_acceleration_rate(
    readAttribute<Double>("maxAccelerationRate", node, scope, Double::infinity())),
  max_deceleration(readAttribute<Double>("maxDeceleration", node, scope, Double::infinity())),
  max_deceleration_rate(
    readAttribute<Double>("maxDecelerationRate", node, scope, Double::infinity())),
  max_speed(readAttribute<Double>("maxSpeed", node, scope, Double::infinity()))
{
}

Performance::operator traffic_simulator_msgs::msg::Performance() const
{
  traffic_simulator_msgs::msg::Performance performance;
  performance.max_acceleration = max_acceleration;
  performance.max_acceleration_rate = max_acceleration_rate;
  performance.max_deceleration = max_deceleration;
  performance.max_deceleration_rate = max_deceleration_rate;
  performance.max_speed = max_speed;
  return performance;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
