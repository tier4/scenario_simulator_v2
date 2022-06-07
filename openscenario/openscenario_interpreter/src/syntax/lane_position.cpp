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

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/lane_position.hpp>
#include <traffic_simulator/helper/helper.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
LanePosition::LanePosition(const pugi::xml_node & node, Scope & scope)
: road_id(readAttribute<String>("roadId", node, scope, "none")),
  lane_id(readAttribute<String>("laneId", node, scope)),
  offset(readAttribute<Double>("offset", node, scope, Double())),
  s(readAttribute<Double>("s", node, scope)),
  orientation(readElement<Orientation>("Orientation", node, scope))
{
}

LanePosition::operator traffic_simulator_msgs::msg::LaneletPose() const
{
  const geometry_msgs::msg::Vector3 rpy = orientation;
  return traffic_simulator::helper::constructLaneletPose(
    static_cast<Integer>(lane_id), s, offset, rpy.x, rpy.y, rpy.z);
}

LanePosition::operator geometry_msgs::msg::Pose() const
{
  return toWorldPosition(static_cast<traffic_simulator_msgs::msg::LaneletPose>(*this));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
