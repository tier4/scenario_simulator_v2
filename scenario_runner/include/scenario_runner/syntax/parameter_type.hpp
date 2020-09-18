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

#ifndef SCENARIO_RUNNER__SYNTAX__PARAMETER_TYPE_HPP_
#define SCENARIO_RUNNER__SYNTAX__PARAMETER_TYPE_HPP_

#include <scenario_runner/syntax/boolean.hpp>
#include <scenario_runner/syntax/double.hpp>
#include <scenario_runner/syntax/integer.hpp>
#include <scenario_runner/syntax/string.hpp>
#include <scenario_runner/syntax/unsigned_integer.hpp>
#include <scenario_runner/syntax/unsigned_short.hpp>

#include <string>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== ParameterType ========================================================
 *
 * <xsd:simpleType name="ParameterType">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="integer"/>
 *         <xsd:enumeration value="double"/>
 *         <xsd:enumeration value="string"/>
 *         <xsd:enumeration value="unsignedInt"/>
 *         <xsd:enumeration value="unsignedShort"/>
 *         <xsd:enumeration value="boolean"/>
 *         <xsd:enumeration value="dateTime"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * ======================================================================== */
struct ParameterType
{
  enum value_type
  {
    INTEGER,
    DOUBLE,
    STRING,
    UNSIGNED_INT,
    UNSIGNED_SHORT,
    BOOLEAN,
    DATE_TIME,
  } value;

  explicit constexpr ParameterType(value_type value = {})
  : value{value}
  {}

  constexpr operator value_type() const noexcept
  {
    return value;
  }
};

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(std::basic_istream<Ts...> & is, ParameterType & type)
{
  std::string buffer {};

  is >> buffer;

  #define SUPPORTED(NAME, IDENTIFIER) \
  if (buffer == NAME) { \
    type.value = ParameterType::IDENTIFIER; \
    return is; \
  } static_assert(true, "")

  SUPPORTED("integer", INTEGER);
  SUPPORTED("double", DOUBLE);
  SUPPORTED("string", STRING);
  SUPPORTED("unsignedInt", UNSIGNED_INT);
  SUPPORTED("unsignedShort", UNSIGNED_SHORT);
  SUPPORTED("boolean", BOOLEAN);
  SUPPORTED("dateTime", DATE_TIME);

  #undef SUPPORTED

  std::stringstream ss {};
  ss << "unexpected value \'" << buffer << "\' specified as type ParameterType";
  throw SyntaxError {ss.str()};
}

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const ParameterType & type)
{
  switch (type) {
    #define BOILERPLATE(NAME, ID) case ParameterType::ID: return os << NAME;

    BOILERPLATE("integer", INTEGER);
    BOILERPLATE("double", DOUBLE);
    BOILERPLATE("string", STRING);
    BOILERPLATE("unsignedInt", UNSIGNED_INT);
    BOILERPLATE("unsignedShort", UNSIGNED_SHORT);
    BOILERPLATE("boolean", BOOLEAN);
    BOILERPLATE("dateTime", DATE_TIME);

    #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class ParameterType holds unexpected value " <<
        static_cast<ParameterType::value_type>(type.value);
      throw ImplementationFault {ss.str()};
  }
}
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__PARAMETER_TYPE_HPP_
