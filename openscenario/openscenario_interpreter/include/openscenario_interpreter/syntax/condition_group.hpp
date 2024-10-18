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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__CONDITION_GROUP_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__CONDITION_GROUP_HPP_

#include <boost/json.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/condition.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ConditionGroup ---------------------------------------------------------
 *
 *  A condition group is an association of conditions that is assessed during
 *  simulation time and signals true when all associated conditions are
 *  evaluated to true.
 *
 *  <xsd:complexType name="ConditionGroup">
 *    <xsd:sequence>
 *      <xsd:element name="Condition" type="Condition" maxOccurs="unbounded"/>
 *    </xsd:sequence>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ConditionGroup : public std::list<Condition>
{
  bool current_value;

  // NOTE: Default constructed ConditionGroup must be return TRUE.
  ConditionGroup() = default;

  explicit ConditionGroup(const pugi::xml_node &, Scope &);

  auto evaluate() -> Object;
};

auto operator<<(boost::json::object &, const ConditionGroup &) -> boost::json::object &;

template <typename T>
using isConditionGroup = typename std::is_same<typename std::decay<T>::type, ConditionGroup>;

static_assert(isConditionGroup<ConditionGroup>::value);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CONDITION_GROUP_HPP_
