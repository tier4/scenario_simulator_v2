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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PEDESTRIAN_CATEGORY_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PEDESTRIAN_CATEGORY_HPP_

#include <openscenario_interpreter/object.hpp>

#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ==== PedestrianCategory ======================================================
 *
 * <xsd:simpleType name="PedestrianCategory">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="pedestrian"/>
 *         <xsd:enumeration value="wheelchair"/>
 *         <xsd:enumeration value="animal"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * ======================================================================== */
struct PedestrianCategory
{
  enum value_type
  {
    pedestrian, wheelchair, animal,
  } value;

  explicit constexpr PedestrianCategory(value_type value = {})
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
  PedestrianCategory & category)
{
  std::string buffer {};

  is >> buffer;

  #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) { \
    category.value = PedestrianCategory::IDENTIFIER; \
    return is; \
  } static_assert(true, "")

  BOILERPLATE(pedestrian);

  #undef BOILERPLATE

  #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) { \
    std::stringstream ss { \
    }; \
    ss << "given value \'" << buffer << \
      "\' is valid OpenSCENARIO value of type PedestrianCategory, but it is not supported"; \
    throw ImplementationFault {ss.str()}; \
  } static_assert(true, "")

  BOILERPLATE(wheelchair);
  BOILERPLATE(animal);

  #undef BOILERPLATE

  std::stringstream ss {};
  ss << "unexpected value \'" << buffer << "\' specified as type PedestrianCategory";
  throw SyntaxError {ss.str()};
}

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(
  std::basic_ostream<Ts...> & os,
  const PedestrianCategory & category)
{
  switch (category) {
    #define BOILERPLATE(NAME) case PedestrianCategory::NAME: return os << #NAME;

    BOILERPLATE(pedestrian);
    BOILERPLATE(wheelchair);
    BOILERPLATE(animal);

    #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class PedestrianCategory holds unexpected value " <<
        static_cast<PedestrianCategory::value_type>(category);
      throw ImplementationFault {ss.str()};
  }
}
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PEDESTRIAN_CATEGORY_HPP_
