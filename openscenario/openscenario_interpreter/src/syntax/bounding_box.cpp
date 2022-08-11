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
#include <openscenario_interpreter/syntax/bounding_box.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
BoundingBox::BoundingBox(const pugi::xml_node & node, Scope & scope)
: center(readElement<Center>("Center", node, scope)),
  dimensions(readElement<Dimensions>("Dimensions", node, scope))
{
}

BoundingBox::operator traffic_simulator_msgs::msg::BoundingBox() const
{
  traffic_simulator_msgs::msg::BoundingBox bounding_box;
  {
    bounding_box.center = static_cast<geometry_msgs::msg::Point>(center);
    bounding_box.dimensions = static_cast<geometry_msgs::msg::Vector3>(dimensions);
  }

  return bounding_box;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
