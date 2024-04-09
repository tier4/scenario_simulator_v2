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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TRIGGERING_ENTITIES_RULE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TRIGGERING_ENTITIES_RULE_HPP_

#include <algorithm>
#include <iostream>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- TriggeringEntitiesRule -------------------------------------------------
 *
 *  <xsd:simpleType name="TriggeringEntitiesRule">
 *    <xsd:union>
 *      <xsd:simpleType>
 *        <xsd:restriction base="xsd:string">
 *          <xsd:enumeration value="any"/>
 *          <xsd:enumeration value="all"/>
 *        </xsd:restriction>
 *      </xsd:simpleType>
 *      <xsd:simpleType>
 *        <xsd:restriction base="parameter"/>
 *      </xsd:simpleType>
 *    </xsd:union>
 *  </xsd:simpleType>
 *
 * -------------------------------------------------------------------------- */
struct TriggeringEntitiesRule
{
  enum value_type {
    all,
    any,
    none,
  } value;

  TriggeringEntitiesRule() = default;

  constexpr operator value_type() const noexcept { return value; }

  template <typename... Ts>
  constexpr auto apply(Ts &&... xs) const -> decltype(auto)
  {
    switch (value) {
      case all:
        return std::all_of(std::forward<decltype(xs)>(xs)...);

      case any:
        return std::any_of(std::forward<decltype(xs)>(xs)...);

      case none:
        return std::none_of(std::forward<decltype(xs)>(xs)...);

      default:
        return false;
    }
  }

  auto description() const -> std::string;
};

auto operator>>(std::istream &, TriggeringEntitiesRule &) -> std::istream &;

auto operator<<(std::ostream &, const TriggeringEntitiesRule &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRIGGERING_ENTITIES_RULE_HPP_
