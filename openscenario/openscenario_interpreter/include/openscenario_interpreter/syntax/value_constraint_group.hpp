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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__VALUE_CONSTRAINT_GROUP_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__VALUE_CONSTRAINT_GROUP_HPP_

#include <nlohmann/json.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/value_constraint.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ValueConstraintGroup ---------------------------------------------------------
 *
 *  A ValueConstraintGroup represents a set of logical constraints which need to evaluate to true
 *  for a defined parameter value to start the simulation.
 *  A constraint group needs to have at least one constraint.
 *  Multiple constraint groups are combined by an OR.
 *
 *
 *  <xsd:complexType name="ValueConstraintGroup">
 *    <xsd:sequence>
 *      <xsd:element name="ValueConstraint" type="ValueConstraint" maxOccurs="unbounded"/>
 *    </xsd:sequence>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ValueConstraintGroup : public std::list<ValueConstraint>
{
  ValueConstraintGroup() = default;

  explicit ValueConstraintGroup(const pugi::xml_node &, Scope &);

  auto evaluate(const Object &) const -> bool;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__VALUE_CONSTRAINT_GROUP_HPP_
