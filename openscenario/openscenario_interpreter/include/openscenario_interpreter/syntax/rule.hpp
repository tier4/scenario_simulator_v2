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

#include <openscenario_interpreter/functional/equal_to.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>

#include <functional>
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
  enum value_type
  {
    greaterThan, lessThan, equalTo,
  } value;

  explicit constexpr Rule(value_type value = {})
  : value{value}
  {}

  constexpr operator value_type() const noexcept
  {
    return value;
  }

  template<typename T, typename U = T>
  constexpr decltype(auto) operator()(const T & lhs, const U & rhs) const noexcept
  {
    switch (value) {
      case greaterThan:
        return std::greater<void>()(
          std::forward<decltype(lhs)>(lhs),
          std::forward<decltype(rhs)>(rhs));

      case lessThan:
        return std::less<void>()(
          std::forward<decltype(lhs)>(lhs),
          std::forward<decltype(rhs)>(rhs));

      case equalTo:
        return equal_to<T>()(
          std::forward<decltype(lhs)>(lhs),
          std::forward<decltype(rhs)>(rhs));

      default:
        return false;
    }
  }
};

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(std::basic_istream<Ts...> & is, Rule & rule)
{
  std::string buffer {};

  is >> buffer;

  #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) { \
    rule.value = Rule::IDENTIFIER; \
    return is; \
  } static_assert(true, "")

  BOILERPLATE(greaterThan);
  BOILERPLATE(lessThan);
  BOILERPLATE(equalTo);

  #undef BOILERPLATE

  std::stringstream ss {};
  ss << "unexpected value \'" << buffer << "\' specified as type Rule";
  throw SyntaxError {ss.str()};
}

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const Rule & rule)
{
  switch (rule) {
    #define BOILERPLATE(ID) case Rule::ID: return os << #ID;

    BOILERPLATE(greaterThan);
    BOILERPLATE(lessThan);
    BOILERPLATE(equalTo);

    #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class Rule holds unexpected value " << static_cast<Rule::value_type>(rule.value);
      throw ImplementationFault {ss.str()};
  }
}
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__RULE_HPP_
