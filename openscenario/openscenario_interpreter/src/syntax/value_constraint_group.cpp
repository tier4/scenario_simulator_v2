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
#include <openscenario_interpreter/syntax/value_constraint_group.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
ValueConstraintGroup::ValueConstraintGroup(const pugi::xml_node & node, Scope & scope)
{
  traverse<1, unbounded>(node, "ValueConstraint", [&](auto && node) { emplace_back(node, scope); });
}
ValueConstraintGroup::ValueConstraintGroup(const openscenario_msgs::msg::ValueConstraintGroup & message)
{
  for (auto & constraint : message.constraints) {
    emplace_back(constraint);
  }
}

auto ValueConstraintGroup::evaluate(const Object & value) const -> bool
{
  auto ret = std::all_of(std::begin(*this), std::end(*this), [&](auto && constraint) {
    return constraint.evaluate(value);
  });
  return ret;
}

}  // namespace syntax
}  // namespace openscenario_interpreter
