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
  reference(readAttribute<EntityRef>("entityRef", node, scope)),
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
  // get world origin pose
  geometry_msgs::msg::Pose original_pose{};
  original_pose.position.x = 0;
  original_pose.position.y = 0;
  original_pose.position.z = 0;
  original_pose.orientation.x = 0;
  original_pose.orientation.y = 0;
  original_pose.orientation.z = 0;
  original_pose.orientation.w = 1;

  // get entity reference pose in world pose
  const auto reference_pose = makeNativeRelativeWorldPosition(original_pose, reference);

  // get offset point in reference pose
  geometry_msgs::msg::Point point_in_reference{};
  point_in_reference.x = dx;
  point_in_reference.y = dy;
  point_in_reference.z = dz;

  // calculate offset pose in world pose
  const auto point_in_world = math::geometry::transformPoint(reference_pose, point_in_reference);
  geometry_msgs::msg::Pose pose_in_world{};
  pose_in_world.position = point_in_world;
  pose_in_world.orientation = original_pose.orientation;

  return pose_in_world;
}

}  // namespace syntax
}  // namespace openscenario_interpreter
