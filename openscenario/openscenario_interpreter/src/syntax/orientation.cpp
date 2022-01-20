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
#include <openscenario_interpreter/syntax/orientation.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_default_constructible<ReferenceContext>::value);
static_assert(std::is_default_constructible<Double>::value);
static_assert(std::is_default_constructible<Orientation>::value);

Orientation::Orientation(const pugi::xml_node & node, Scope & scope)
: type(readAttribute<ReferenceContext>("type", node, scope, ReferenceContext())),
  h(readAttribute<Double>("h", node, scope, Double())),
  p(readAttribute<Double>("p", node, scope, Double())),
  r(readAttribute<Double>("r", node, scope, Double()))
{
}

Orientation::operator geometry_msgs::msg::Vector3() const
{
  geometry_msgs::msg::Vector3 result{};

  switch (type) {
    case ReferenceContext::relative:
      result.x = r;  // roll
      result.y = p;  // pitch
      result.z = h;  // yaw (heading)
      break;

    case ReferenceContext::absolute:
      // Jumps can never reach here (see ReferenceContext::operator >>).

    default:
      break;
  }

  return result;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
