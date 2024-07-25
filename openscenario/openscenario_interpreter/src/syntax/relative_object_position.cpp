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

#include <geometry/transform.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/relative_object_position.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
RelativeObjectPosition::RelativeObjectPosition(const pugi::xml_node & node, Scope & scope)
: orientation(readElement<Orientation>("Orientation", node, scope)),
  entity_ref(readAttribute<String>("entityRef", node, scope), scope),
  dx(readAttribute<Double>("dx", node, scope)),
  dy(readAttribute<Double>("dy", node, scope)),
  dz(readAttribute<Double>("dz", node, scope, Double()))
{
}

RelativeObjectPosition::operator geometry_msgs::msg::Point() const
{
  geometry_msgs::msg::Point result;
  {
    result.x = dx;
    result.y = dy;
    result.z = dz;
  }

  return result;
}

RelativeObjectPosition::operator NativeLanePosition() const
{
  return convert<NativeLanePosition>(static_cast<NativeWorldPosition>(*this));
}

RelativeObjectPosition::operator NativeWorldPosition() const
{
  geometry_msgs::msg::Pose world_origin_pose{};
  world_origin_pose.position.x = 0;
  world_origin_pose.position.y = 0;
  world_origin_pose.position.z = 0;
  world_origin_pose.orientation.x = 0;
  world_origin_pose.orientation.y = 0;
  world_origin_pose.orientation.z = 0;
  world_origin_pose.orientation.w = 1;

  const auto reference_pose_in_world =
    makeNativeRelativeWorldPosition(world_origin_pose, entity_ref);

  geometry_msgs::msg::Point target_point_in_entity_ref{};
  target_point_in_entity_ref.x = dx;
  target_point_in_entity_ref.y = dy;
  target_point_in_entity_ref.z = dz;

  geometry_msgs::msg::Pose target_pose_in_world{};
  target_pose_in_world.position =
    math::geometry::transformPoint(reference_pose_in_world, target_point_in_entity_ref);
  target_pose_in_world.orientation = world_origin_pose.orientation;

  return target_pose_in_world;
}

}  // namespace syntax
}  // namespace openscenario_interpreter
