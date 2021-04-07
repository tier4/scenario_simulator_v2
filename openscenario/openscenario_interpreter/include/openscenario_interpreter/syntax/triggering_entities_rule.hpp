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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TRIGGERING_ENTITIES_RULE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TRIGGERING_ENTITIES_RULE_HPP_

#include <openscenario_interpreter/object.hpp>
#include <string>
#include <utility>
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ==== TriggeringEntitiesRule ===============================================
 *
 * <xsd:simpleType name="TriggeringEntitiesRule">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="any"/>
 *         <xsd:enumeration value="all"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * ======================================================================== */
struct TriggeringEntitiesRule
{
  enum value_type {
    all,
    any,
    none,
  } value;

  explicit constexpr TriggeringEntitiesRule(value_type value = {}) : value{value} {}

  constexpr operator value_type() const noexcept { return value; }

  template <typename... Ts>
  constexpr decltype(auto) operator()(Ts &&... xs) const
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
};

template <typename... Ts>
std::basic_istream<Ts...> & operator>>(
  std::basic_istream<Ts...> & is, TriggeringEntitiesRule & rule)
{
  std::string buffer{};

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)                      \
  if (buffer == #IDENTIFIER) {                       \
    rule.value = TriggeringEntitiesRule::IDENTIFIER; \
    return is;                                       \
  }                                                  \
  static_assert(true, "")

  BOILERPLATE(all);
  BOILERPLATE(any);

#undef BOILERPLATE

  std::stringstream ss{};
  ss << "unexpected value \'" << buffer << "\' specified as type TriggeringEntitiesRule";
  throw SyntaxError{ss.str()};
}

template <typename... Ts>
std::basic_ostream<Ts...> & operator<<(
  std::basic_ostream<Ts...> & os, const TriggeringEntitiesRule & rule)
{
  switch (rule) {
#define BOILERPLATE(ID)            \
  case TriggeringEntitiesRule::ID: \
    return os << #ID;

    BOILERPLATE(all);
    BOILERPLATE(any);

#undef BOILERPLATE

    default:
      std::stringstream ss{};
      ss << "enum class TriggeringEntitiesRule holds unexpected value "
         << static_cast<TriggeringEntitiesRule::value_type>(rule.value);
      throw ImplementationFault{ss.str()};
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRIGGERING_ENTITIES_RULE_HPP_
