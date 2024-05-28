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
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/lane_position.hpp>

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

LanePosition::LanePosition(
  const String & road_id, const String & lane_id, const Double & offset, const Double & s,
  const Orientation & orientation)
: road_id(road_id), lane_id(lane_id), offset(offset), s(s), orientation(orientation)
{
}

LanePosition::operator NativeLanePosition() const
{
  traffic_simulator::LaneletPose native_lane_position;
  native_lane_position.lanelet_id = boost::lexical_cast<std::int64_t>(lane_id);
  native_lane_position.s = s;
  native_lane_position.offset = offset;
  native_lane_position.rpy.x = orientation.r;
  native_lane_position.rpy.y = orientation.p;
  native_lane_position.rpy.z = orientation.h;
  return canonicalize(native_lane_position);
}

LanePosition::operator NativeWorldPosition() const
{
  return convert<NativeWorldPosition>(static_cast<NativeLanePosition>(*this));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
