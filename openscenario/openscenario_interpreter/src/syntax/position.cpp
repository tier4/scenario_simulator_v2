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
#include <openscenario_interpreter/syntax/position.hpp>
#include <openscenario_interpreter/utility/overload.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Position::Position(const pugi::xml_node & node, Scope & scope)
// clang-format off
: ComplexType(
    choice(node,
      std::make_pair(         "WorldPosition", [&](auto && node) { return make<         WorldPosition>(node, scope); }),
      std::make_pair( "RelativeWorldPosition", [&](auto && node) { return make< RelativeWorldPosition>(node, scope); }),
      std::make_pair("RelativeObjectPosition", [&](auto && node) { return make<RelativeObjectPosition>(node, scope); }),
      std::make_pair(          "RoadPosition", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
      std::make_pair(  "RelativeRoadPosition", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
      std::make_pair(          "LanePosition", [&](auto && node) { return make<         LanePosition>(node, scope); }),
      std::make_pair(  "RelativeLanePosition", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
      std::make_pair(         "RoutePosition", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
      std::make_pair(           "GeoPosition", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
      std::make_pair(    "TrajectoryPosition", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; })))
// clang-format on
{
}

Position::operator geometry_msgs::msg::Pose() const
{
  return apply<geometry_msgs::msg::Pose>(
    overload(
      // clang-format off
      [&](const          WorldPosition & position) { return static_cast<geometry_msgs::msg::Pose>(position); },
      [&](const  RelativeWorldPosition & position) { return static_cast<geometry_msgs::msg::Pose>(position); },
      [&](const RelativeObjectPosition & position) { return static_cast<geometry_msgs::msg::Pose>(position); },
      [&](const           LanePosition & position) { return static_cast<geometry_msgs::msg::Pose>(position); }
      // clang-format on
      ),
    *this);
}

Position::operator NativeLanePosition() const
{
  return apply<NativeLanePosition>(
    overload(
      // clang-format off
      [&](const         WorldPosition  & position) { return static_cast<NativeLanePosition>(position); },
      [&](const RelativeWorldPosition  & position) { return static_cast<NativeLanePosition>(position); },
      [&](const RelativeObjectPosition & position) { return static_cast<NativeLanePosition>(position); },
      [&](const          LanePosition  & position) { return static_cast<NativeLanePosition>(position); }
      // clang-format on
      ),
    *this);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
