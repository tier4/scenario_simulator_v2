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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__VEHICLE_CATEGORY_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__VEHICLE_CATEGORY_HPP_

#include <openscenario_interpreter/object.hpp>

#include <string>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ==== VehicleCategory ======================================================
 *
 * <xsd:simpleType name="VehicleCategory">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="car"/>
 *         <xsd:enumeration value="van"/>
 *         <xsd:enumeration value="truck"/>
 *         <xsd:enumeration value="trailer"/>
 *         <xsd:enumeration value="semitrailer"/>
 *         <xsd:enumeration value="bus"/>
 *         <xsd:enumeration value="motorbike"/>
 *         <xsd:enumeration value="bicycle"/>
 *         <xsd:enumeration value="train"/>
 *         <xsd:enumeration value="tram"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * ======================================================================== */
struct VehicleCategory
{
  enum value_type
  {
    bicycle,
    bus,
    car,
    motorbike,
    semitrailer,
    trailer,
    train,
    tram,
    truck,
    van,
  } value;

  explicit constexpr VehicleCategory(value_type value = {})
  : value{value}
  {}

  constexpr operator value_type() const noexcept
  {
    return value;
  }
};

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(std::basic_istream<Ts...> & is, VehicleCategory & category)
{
  std::string buffer {};

  is >> buffer;

  #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) { \
    category.value = VehicleCategory::IDENTIFIER; \
    return is; \
  } static_assert(true, "")

  BOILERPLATE(bicycle);
  BOILERPLATE(bus);
  BOILERPLATE(car);
  BOILERPLATE(motorbike);
  BOILERPLATE(truck);

  #undef BOILERPLATE

  #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) { \
    std::stringstream ss { \
    }; \
    ss << "given value \'" << buffer << \
      "\' is valid OpenSCENARIO value of type VehicleCategory, but it is not supported"; \
    throw ImplementationFault {ss.str()}; \
  } static_assert(true, "")

  BOILERPLATE(semitrailer);
  BOILERPLATE(trailer);
  BOILERPLATE(train);
  BOILERPLATE(tram);
  BOILERPLATE(van);

  #undef BOILERPLATE

  std::stringstream ss {};
  ss << "unexpected value \'" << buffer << "\' specified as type VehicleCategory";
  throw SyntaxError {ss.str()};
}

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(
  std::basic_ostream<Ts...> & os,
  const VehicleCategory & category)
{
  switch (category) {
    #define BOILERPLATE(NAME) case VehicleCategory::NAME: return os << #NAME;

    BOILERPLATE(bicycle);
    BOILERPLATE(bus);
    BOILERPLATE(car);
    BOILERPLATE(motorbike);
    BOILERPLATE(semitrailer);
    BOILERPLATE(trailer);
    BOILERPLATE(train);
    BOILERPLATE(tram);
    BOILERPLATE(truck);
    BOILERPLATE(van);

    #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class VehicleCategory holds unexpected value " <<
        static_cast<VehicleCategory::value_type>(category);
      throw ImplementationFault {ss.str()};
  }
}
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__VEHICLE_CATEGORY_HPP_
