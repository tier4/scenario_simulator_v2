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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__DYNAMICS_SHAPE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__DYNAMICS_SHAPE_HPP_

#include <openscenario_interpreter/object.hpp>

#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ==== DynamicsShape ========================================================
 *
 * <xsd:simpleType name="DynamicsShape">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="linear"/>
 *         <xsd:enumeration value="cubic"/>
 *         <xsd:enumeration value="sinusoidal"/>
 *         <xsd:enumeration value="step"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * ======================================================================== */
struct DynamicsShape
{
  enum value_type
  {
    // Value changes in a linear function: f(x) = f_0 + rate * x.
    linear,

    // Cubical transition f(x)=A*x^3+B*x^2+C*x+D with the constraint that the
    // gradient must be zero at start and end.
    cubic,

    // Sinusoidal transition f(x)=A*sin(x)+B with the constraint that the
    // gradient must be zero at start and end.
    sinusoidal,

    // Step transition.
    step,
  } value;

  constexpr DynamicsShape(value_type value = {})
  : value{value}
  {}

  constexpr operator value_type() const noexcept
  {
    return value;
  }
};

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(std::basic_istream<Ts...> & is, DynamicsShape & shape)
{
  std::string buffer {};

  is >> buffer;

  #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) { \
    shape = DynamicsShape::IDENTIFIER; \
    return is; \
  } static_assert(true, "")

  BOILERPLATE(linear);
  BOILERPLATE(step);

  #undef BOILERPLATE

  #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) { \
    std::stringstream ss { \
    }; \
    ss << "given value \'" << buffer << \
      "\' is valid OpenSCENARIO value of type DynamicsShape, but it is not supported"; \
    throw ImplementationFault {ss.str()}; \
  } static_assert(true, "")

  BOILERPLATE(cubic);
  BOILERPLATE(sinusoidal);

  #undef BOILERPLATE

  std::stringstream ss {};
  ss << "unexpected value \'" << buffer << "\' specified as type DynamicsShape";
  throw SyntaxError {ss.str()};
}

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const DynamicsShape & shape)
{
  switch (shape) {
    #define BOILERPLATE(NAME) case DynamicsShape::NAME: return os << #NAME;

    BOILERPLATE(linear);
    BOILERPLATE(cubic);
    BOILERPLATE(sinusoidal);
    BOILERPLATE(step);

    #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class DynamicsShape holds unexpected value " <<
        static_cast<DynamicsShape::value_type>(shape);
      throw ImplementationFault {ss.str()};
  }
}
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__DYNAMICS_SHAPE_HPP_
