// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__RULE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__RULE_HPP_

#include <functional>
#include <openscenario_interpreter/functional/equal_to.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <string>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Rule -------------------------------------------------------------------
 *
 * <xsd:simpleType name="Rule">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="greaterThan"/>
 *         <xsd:enumeration value="lessThan"/>
 *         <xsd:enumeration value="equalTo"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * -------------------------------------------------------------------------- */
struct Rule
{
  enum value_type {
    greaterThan,
    lessThan,
    equalTo,
  } value;

  explicit Rule() = default;

  explicit Rule(value_type value) : value(value) {}

  constexpr operator value_type() const noexcept { return value; }

  template <typename T, typename U = T>
  constexpr decltype(auto) operator()(const T & lhs, const U & rhs) const noexcept
  {
    switch (value) {
      case greaterThan:
        return std::greater<void>()(
          std::forward<decltype(lhs)>(lhs), std::forward<decltype(rhs)>(rhs));

      case lessThan:
        return std::less<void>()(
          std::forward<decltype(lhs)>(lhs), std::forward<decltype(rhs)>(rhs));

      case equalTo:
        return equal_to<T>()(std::forward<decltype(lhs)>(lhs), std::forward<decltype(rhs)>(rhs));

      default:
        return false;
    }
  }
};

static_assert(std::is_standard_layout<Rule>::value, "");

static_assert(std::is_trivial<Rule>::value, "");

std::istream & operator>>(std::istream &, Rule &);

std::ostream & operator<<(std::ostream &, const Rule &);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__RULE_HPP_
