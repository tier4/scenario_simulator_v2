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

#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/operator.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/relative_world_position.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
RelativeWorldPosition::RelativeWorldPosition(const pugi::xml_node & node, Scope & scope)
: orientation(readElement<Orientation>("Orientation", node, scope)),
  entity_ref(readAttribute<String>("entityRef", node, scope), scope),
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

RelativeWorldPosition::operator NativeLanePosition() const
{
  return toNativeLanePosition(static_cast<NativeWorldPosition>(*this));
}

RelativeWorldPosition::operator NativeWorldPosition() const
{
  auto world_origin_pose = geometry_msgs::msg::Pose();
  world_origin_pose.orientation.w = 1;

  const auto reference_pose_in_world =
    makeNativeRelativeWorldPosition(world_origin_pose, entity_ref);

  using math::geometry::operator*;

  auto target_pose_in_world = geometry_msgs::msg::Pose();
  target_pose_in_world.position.x = reference_pose_in_world.position.x + dx;
  target_pose_in_world.position.y = reference_pose_in_world.position.y + dy;
  target_pose_in_world.position.z = reference_pose_in_world.position.z + dz;
  target_pose_in_world.orientation =
    reference_pose_in_world.orientation * math::geometry::convertEulerAngleToQuaternion(
                                            static_cast<geometry_msgs::msg::Vector3>(orientation));
  return target_pose_in_world;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
