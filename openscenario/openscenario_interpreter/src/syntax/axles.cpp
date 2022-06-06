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

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/axles.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Axles::Axles(const pugi::xml_node & node, Scope & scope)
: front_axle(readElement<FrontAxle>("FrontAxle", node, scope)),
  rear_axle(readElement<RearAxle>("RearAxle", node, scope)),
  additional_axles(readElements<AdditionalAxle, 0>("AdditionalAxle", node, scope))
{
}

Axles::operator traffic_simulator_msgs::msg::Axles() const
{
  traffic_simulator_msgs::msg::Axles axles;
  {
    axles.front_axle = static_cast<traffic_simulator_msgs::msg::Axle>(front_axle);
    axles.rear_axle = static_cast<traffic_simulator_msgs::msg::Axle>(rear_axle);
  }

  return axles;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
