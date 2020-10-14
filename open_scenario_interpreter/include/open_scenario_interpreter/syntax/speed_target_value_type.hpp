// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__SPEED_TARGET_VALUE_TYPE_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__SPEED_TARGET_VALUE_TYPE_HPP_

#include <open_scenario_interpreter/object.hpp>

#include <string>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ==== SpeedTargetValueType =================================================
 *
 * <xsd:simpleType name="SpeedTargetValueType">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="delta"/>
 *         <xsd:enumeration value="factor"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * ======================================================================== */
struct SpeedTargetValueType
{
  enum value_type
  {
    // The relative value is interpreted as a difference to a referenced value.
    // Unit: m/s. As an example, a speed value of 10 equals a speed that's 10m/s
    // faster than the reference speed.
    delta,

    // The relative value is interpreted as a factor to a referenced value. No
    // unit. As an example, a speed value of 1.1 equals a speed that's 10%
    // faster than the reference speed.
    factor,
  } value;

  explicit constexpr SpeedTargetValueType(value_type value = delta)
  : value{value}
  {}

  constexpr operator value_type() const noexcept
  {
    return value;
  }
};

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(std::basic_istream<Ts...> & is, SpeedTargetValueType & type)
{
  std::string buffer {};

  is >> buffer;

  #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) { \
    type.value = SpeedTargetValueType::IDENTIFIER; \
    return is; \
  } static_assert(true, "")

  BOILERPLATE(delta);

  #undef BOILERPLATE

  #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) { \
    std::stringstream ss { \
    }; \
    ss << "given value \'" \
       << buffer \
       << "\' is valid OpenSCENARIO value of type SpeedTargetValueType, but it is not supported"; \
    throw ImplementationFault {ss.str()}; \
  } static_assert(true, "")

  BOILERPLATE(factor);

  #undef BOILERPLATE

  std::stringstream ss {};
  ss << "unexpected value \'" << buffer << "\' specified as type SpeedTargetValueType";
  throw SyntaxError {ss.str()};
}

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(
  std::basic_ostream<Ts...> & os,
  const SpeedTargetValueType & type)
{
  switch (type) {
    #define BOILERPLATE(NAME) case SpeedTargetValueType::NAME: return os << #NAME;

    BOILERPLATE(delta);
    BOILERPLATE(factor);

    #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class SpeedTargetValueType holds unexpected value " <<
        static_cast<SpeedTargetValueType::value_type>(type);
      throw ImplementationFault {ss.str()};
  }
}
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__SPEED_TARGET_VALUE_TYPE_HPP_
