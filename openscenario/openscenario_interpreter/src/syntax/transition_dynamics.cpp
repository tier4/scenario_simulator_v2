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

#include <openscenario_interpreter/syntax/transition_dynamics.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
TransitionDynamics::TransitionDynamics(const pugi::xml_node & node, Scope & scope)
: dynamics_shape(readAttribute<DynamicsShape>("dynamicsShape", node, scope)),
  value(readAttribute<Double>("value", node, scope)),
  dynamics_dimension(readAttribute<DynamicsDimension>("dynamicsDimension", node, scope))
{
}
}  // namespace syntax
}  // namespace openscenario_interpreter
