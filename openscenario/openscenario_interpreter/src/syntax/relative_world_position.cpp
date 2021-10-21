// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/relative_world_position.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
RelativeWorldPosition::RelativeWorldPosition(const pugi::xml_node & node, Scope & scope)
: orientation(readElement<Orientation>("Orientation", node, scope)),
  reference(readAttribute<EntityRef>("entityRef", node, scope)),
  dx(readAttribute<Double>("dx", node, scope)),
  dy(readAttribute<Double>("dy", node, scope)),
  dz(readAttribute<Double>("dz", node, scope, Double()))
{
}

RelativeWorldPosition::operator geometry_msgs::msg::Point() const
{
  geometry_msgs::msg::Point result;
  {
    result.x = dx;
    result.y = dy;
    result.z = dz;
  }

  return result;
}

RelativeWorldPosition::operator geometry_msgs::msg::Pose() const
{
  throw UNSUPPORTED_CONVERSION_DETECTED(RelativeWorldPosition, geometry_msgs::msg::Pose);
}

RelativeWorldPosition::operator traffic_simulator_msgs::msg::LaneletPose() const
{
  throw UNSUPPORTED_CONVERSION_DETECTED(
    RelativeWorldPosition, traffic_simulator_msgs::msg::LaneletPose);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
