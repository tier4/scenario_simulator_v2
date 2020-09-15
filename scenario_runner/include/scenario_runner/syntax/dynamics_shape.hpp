#ifndef SCENARIO_RUNNER__SYNTAX__DYNAMICS_SHAPE_HPP_
#define SCENARIO_RUNNER__SYNTAX__DYNAMICS_SHAPE_HPP_

#include <scenario_runner/object.hpp>

namespace scenario_runner
{inline namespace syntax
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

    #define SUPPORTED(IDENTIFIER) \
  if (buffer == #IDENTIFIER) do \
    { \
      shape = DynamicsShape::IDENTIFIER; \
      return is; \
    } while (false)

  SUPPORTED(linear);
  SUPPORTED(step);

    #undef SUPPORTED

    #define UNSUPPORTED(IDENTIFIER) \
  if (buffer == #IDENTIFIER) do \
    { \
      std::stringstream ss {}; \
      ss << "given value \'" << buffer << \
        "\' is valid OpenSCENARIO value of type DynamicsShape, but it is not supported"; \
      throw ImplementationFault {ss.str()}; \
    } while (false)

  UNSUPPORTED(cubic);
  UNSUPPORTED(sinusoidal);

    #undef UNSUPPORTED

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
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__DYNAMICS_SHAPE_HPP_
