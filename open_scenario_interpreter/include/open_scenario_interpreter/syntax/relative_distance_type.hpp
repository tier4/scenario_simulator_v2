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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__RELATIVE_DISTANCE_TYPE_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__RELATIVE_DISTANCE_TYPE_HPP_

#include <string>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ==== RelativeDistanceType =================================================
 *
 * <xsd:simpleType name="RelativeDistanceType">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="longitudinal"/>
 *         <xsd:enumeration value="lateral"/>
 *         <xsd:enumeration value="cartesianDistance"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * ======================================================================== */
struct RelativeDistanceType
{
  enum value_type
  {
    longitudinal,
    lateral,
    cartesianDistance,
  }
  value;

  explicit constexpr RelativeDistanceType(value_type value = {})
  : value{value}
  {}

  constexpr operator value_type() const noexcept
  {
    return value;
  }
};

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(std::basic_istream<Ts...> & is, RelativeDistanceType & type)
{
  std::string buffer {};

  is >> buffer;

  #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) { \
    type.value = RelativeDistanceType::IDENTIFIER; \
    return is; \
  } static_assert(true, "")

  BOILERPLATE(longitudinal);
  BOILERPLATE(lateral);
  BOILERPLATE(cartesianDistance);

  #undef BOILERPLATE

  std::stringstream ss {};
  ss << "unexpected value \'" << buffer << "\' specified as type RelativeDistanceType";
  throw SyntaxError {ss.str()};
}

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(
  std::basic_ostream<Ts...> & os,
  const RelativeDistanceType & type)
{
  switch (type) {
    #define BOILERPLATE(ID) case RelativeDistanceType::ID: return os << #ID;

    BOILERPLATE(longitudinal);
    BOILERPLATE(lateral);
    BOILERPLATE(cartesianDistance);

    #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class RelativeDistanceType holds unexpected value " <<
        static_cast<RelativeDistanceType::value_type>(type.value);
      throw ImplementationFault {ss.str()};
  }
}
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__RELATIVE_DISTANCE_TYPE_HPP_
