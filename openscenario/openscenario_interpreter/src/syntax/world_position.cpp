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

#include <quaternion_operation/quaternion_operation.h>

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/world_position.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
WorldPosition::WorldPosition(const pugi::xml_node & node, Scope & scope)
: x(readAttribute<Double>("x", node, scope)),
  y(readAttribute<Double>("y", node, scope)),
  z(readAttribute<Double>("z", node, scope, Double())),
  h(readAttribute<Double>("h", node, scope, Double())),  // yaw
  p(readAttribute<Double>("p", node, scope, Double())),
  r(readAttribute<Double>("r", node, scope, Double()))
{
}

WorldPosition::operator NativeLanePosition() const
{
  return convert<NativeLanePosition>(static_cast<NativeWorldPosition>(*this));
}

WorldPosition::operator NativeWorldPosition() const { return makeNativeWorldPosition(*this); }
}  // namespace syntax
}  // namespace openscenario_interpreter
