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
#include <openscenario_interpreter/syntax/value_constraint_group.hpp>
#include <openscenario_interpreter/utility/compare.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
ValueConstraintGroup::ValueConstraintGroup(const pugi::xml_node & node, Scope & scope)
{
  traverse<1, unbounded>(node, "Constraint", [&](auto && node) { emplace_back(node, scope); });
}
ValueConstraintGroup::ValueConstraintGroup(
  const std::vector<openscenario_msgs::msg::ValueConstraint> & constraints)
{
  for(auto & constraint : constraints){
    emplace_back(constraint);
  }
}

auto evaluate() -> Object {}

}  // namespace syntax
}  // namespace openscenario_interpreter

//auto operator>>(std::istream & is, openscenario_interpreter::syntax::ValueConstraintGroup & datum)
//  -> std::istream &
//{
//  std::string buffer;
//
//  is >> buffer;
//
//
//
//}
//
//auto operator<<(std::ostream & os, const openscenario_interpreter::syntax::ValueConstraintGroup & datum)
//  -> std::ostream &
//{
//}
