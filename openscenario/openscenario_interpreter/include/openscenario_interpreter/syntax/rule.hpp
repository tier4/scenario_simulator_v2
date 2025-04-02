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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__RULE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__RULE_HPP_

#include <functional>
#include <openscenario_interpreter/functional/equal_to.hpp>
#include <string>
#include <utility>
#include <valarray>

namespace openscenario_interpreter
{
inline namespace syntax
{
template <typename T, typename U>
struct RuleResultDeduction
{
  using type = bool;
};

template <typename T, typename U>
struct RuleResultDeduction<std::valarray<T>, U>
{
  using type = std::valarray<bool>;
};
/*
   Rule (OpenSCENARIO XML 1.3)

   <xsd:simpleType name="Rule">
     <xsd:union>
       <xsd:simpleType>
         <xsd:restriction base="xsd:string">
           <xsd:enumeration value="equalTo"/>
           <xsd:enumeration value="greaterThan"/>
           <xsd:enumeration value="lessThan"/>
           <xsd:enumeration value="greaterOrEqual"/>
           <xsd:enumeration value="lessOrEqual"/>
           <xsd:enumeration value="notEqualTo"/>
         </xsd:restriction>
       </xsd:simpleType>
       <xsd:simpleType>
         <xsd:restriction base="parameter"/>
       </xsd:simpleType>
     </xsd:union>
   </xsd:simpleType>
*/
struct Rule
{
  enum value_type {
    equalTo,
    greaterThan,
    lessThan,
    greaterOrEqual,
    lessOrEqual,
    notEqualTo,
  } value;

  Rule() = default;

  explicit Rule(value_type value) : value(value) {}

  constexpr operator value_type() const noexcept { return value; }

  template <typename T, typename U = T>
  constexpr auto operator()(const T & lhs, const U & rhs) const noexcept ->
    typename RuleResultDeduction<T, U>::type
  {
    switch (value) {
      case equalTo:
        return equal_to<T>()(std::forward<decltype(lhs)>(lhs), std::forward<decltype(rhs)>(rhs));
      case greaterThan:
        return std::greater()(std::forward<decltype(lhs)>(lhs), std::forward<decltype(rhs)>(rhs));
      case lessThan:
        return std::less()(std::forward<decltype(lhs)>(lhs), std::forward<decltype(rhs)>(rhs));
      case greaterOrEqual:
        return std::greater_equal()(
          std::forward<decltype(lhs)>(lhs), std::forward<decltype(rhs)>(rhs));
      case lessOrEqual:
        return std::less_equal()(
          std::forward<decltype(lhs)>(lhs), std::forward<decltype(rhs)>(rhs));
      case notEqualTo:
        return std::not_equal_to()(
          std::forward<decltype(lhs)>(lhs), std::forward<decltype(rhs)>(rhs));
      default:
        return {};
    }
  }
};

auto operator>>(std::istream &, Rule &) -> std::istream &;

auto operator<<(std::ostream &, const Rule &) -> std::ostream &;

}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__RULE_HPP_
