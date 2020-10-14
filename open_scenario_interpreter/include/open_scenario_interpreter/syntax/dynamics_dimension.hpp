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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__DYNAMICS_DIMENSION_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__DYNAMICS_DIMENSION_HPP_

#include <open_scenario_interpreter/object.hpp>

#include <string>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ==== DynamicsDimension ====================================================
 *
 * <xsd:simpleType name="DynamicsDimension">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="rate"/>
 *         <xsd:enumeration value="time"/>
 *         <xsd:enumeration value="distance"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * ======================================================================== */
struct DynamicsDimension
{
  enum value_type
  {
    // A predefined constant rate is used to acquire the target value.
    rate,

    // A predefined time (duration) is used to acquire the target value.
    time,

    // A predefined distance used to acquire the target value.
    distance,
  } value;

  constexpr DynamicsDimension(value_type value = {})
  : value{value}
  {}

  constexpr operator value_type() const noexcept
  {
    return value;
  }
};

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(
  std::basic_istream<Ts...> & is,
  DynamicsDimension & dimension)
{
  std::string buffer {};

  is >> buffer;

  #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) { \
    dimension = DynamicsDimension::IDENTIFIER; \
    return is; \
  } static_assert(true, "")

  BOILERPLATE(rate);
  BOILERPLATE(time);
  BOILERPLATE(distance);

  #undef BOILERPLATE

  #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) { \
    std::stringstream ss { \
    }; \
    ss << "given value \'" << buffer << \
      "\' is valid OpenSCENARIO value of type DynamicsDimension, but it is not supported"; \
    throw ImplementationFault {ss.str()}; \
  } static_assert(true, "")

  #undef BOILERPLATE

  std::stringstream ss {};
  ss << "unexpected value \'" << buffer << "\' specified as type DynamicsDimension";
  throw SyntaxError {ss.str()};
}

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(
  std::basic_ostream<Ts...> & os,
  const DynamicsDimension & dimension)
{
  switch (dimension) {
    #define BOILERPLATE(NAME) case DynamicsDimension::NAME: return os << #NAME;

    BOILERPLATE(rate);
    BOILERPLATE(time);
    BOILERPLATE(distance);

    #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class DynamicsDimension holds unexpected value " <<
        static_cast<DynamicsDimension::value_type>(dimension);
      throw ImplementationFault {ss.str()};
  }
}
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__DYNAMICS_DIMENSION_HPP_
