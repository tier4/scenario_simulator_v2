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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__REFERENCE_CONTEXT_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__REFERENCE_CONTEXT_HPP_

#include <openscenario_interpreter/object.hpp>

#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ==== ReferenceContext =====================================================
 *
 * <xsd:simpleType name="ReferenceContext">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="relative"/>
 *         <xsd:enumeration value="absolute"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * ======================================================================== */
struct ReferenceContext
{
  enum value_type
  {
    relative, absolute,
  } value;

  constexpr ReferenceContext(value_type value = {})
  : value{value}
  {}

  constexpr operator value_type() const noexcept
  {
    return value;
  }
};

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(std::basic_istream<Ts...> & is, ReferenceContext & context)
{
  std::string buffer {};

  is >> buffer;

  #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) { \
    context.value = ReferenceContext::IDENTIFIER; \
    return is; \
  } static_assert(true, "")

  BOILERPLATE(relative);

  #undef BOILERPLATE

  #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) { \
    std::stringstream ss { \
    }; \
    ss << "given value \'" << buffer << \
      "\' is valid OpenSCENARIO value of type ReferenceContext, but it is not supported"; \
    throw ImplementationFault {ss.str()}; \
  } static_assert(true, "")

  BOILERPLATE(absolute);

  #undef BOILERPLATE

  std::stringstream ss {};
  ss << "unexpected value \'" << buffer << "\' specified as type ReferenceContext";
  throw SyntaxError {ss.str()};
}

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(
  std::basic_ostream<Ts...> & os,
  const ReferenceContext & context)
{
  switch (context) {
    #define BOILERPLATE(ID) case ReferenceContext::ID: return os << #ID;

    BOILERPLATE(absolute);
    BOILERPLATE(relative);

    #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class ReferenceContext holds unexpected value " <<
        static_cast<ReferenceContext::value_type>(context.value);
      throw ImplementationFault {ss.str()};
  }
}
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__REFERENCE_CONTEXT_HPP_
