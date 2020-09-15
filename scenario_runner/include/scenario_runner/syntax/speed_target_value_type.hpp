#ifndef SCENARIO_RUNNER__SYNTAX__SPEED_TARGET_VALUE_TYPE_HPP_
#define SCENARIO_RUNNER__SYNTAX__SPEED_TARGET_VALUE_TYPE_HPP_

#include <scenario_runner/object.hpp>

namespace scenario_runner
{inline namespace syntax
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

  explicit SpeedTargetValueType() = default;

  SpeedTargetValueType(value_type value)
  : value{value}
  {}

  operator value_type() const noexcept
  {
    return value;
  }
};

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(std::basic_istream<Ts...> & is, SpeedTargetValueType & type)
{
  std::string buffer {};

  is >> buffer;

    #define SUPPORTED(IDENTIFIER) \
  if (buffer == #IDENTIFIER) do \
    { \
      type.value = SpeedTargetValueType::IDENTIFIER; \
      return is; \
    } while (false)

  SUPPORTED(delta);

    #undef SUPPORTED

    #define UNSUPPORTED(IDENTIFIER) \
  if (buffer == #IDENTIFIER) do \
    { \
      std::stringstream ss {}; \
      ss << "given value \'" << buffer << \
        "\' is valid OpenSCENARIO value of type SpeedTargetValueType, but it is not supported"; \
      throw ImplementationFault {ss.str()}; \
    } while (false)

  UNSUPPORTED(factor);

    #undef UNSUPPORTED

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
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__SPEED_TARGET_VALUE_TYPE_HPP_
