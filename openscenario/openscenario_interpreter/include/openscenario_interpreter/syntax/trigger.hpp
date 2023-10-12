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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TRIGGER_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TRIGGER_HPP_

#include <nlohmann/json.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/condition_group.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Trigger ----------------------------------------------------------------
 *
 *  <xsd:complexType name="Trigger">
 *    <xsd:sequence>
 *      <xsd:element name="ConditionGroup" type="ConditionGroup" minOccurs="0" maxOccurs="unbounded"/>
 *    </xsd:sequence>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Trigger : public std::list<ConditionGroup>
{
  bool current_value;

  // NOTE: Default constructed Trigger must be return FALSE.
  Trigger() = default;

  explicit Trigger(const pugi::xml_node &, Scope &);

  explicit Trigger(const std::list<ConditionGroup> & condition_groups)
  : std::list<ConditionGroup>(condition_groups)
  {
  }

  auto activeConditionGroupIndex() const -> iterator::difference_type;

  auto activeConditionGroupDescription() const -> std::vector<std::pair<std::string, std::string>>;

  auto evaluate() -> Object;
};

auto operator<<(nlohmann::json &, const Trigger &) -> nlohmann::json &;

static_assert(std::is_default_constructible<Trigger>::value);

static_assert(std::is_nothrow_default_constructible<Trigger>::value);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRIGGER_HPP_
