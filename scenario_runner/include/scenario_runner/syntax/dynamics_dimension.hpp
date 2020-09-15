#ifndef SCENARIO_RUNNER__SYNTAX__DYNAMICS_DIMENSION_HPP_
#define SCENARIO_RUNNER__SYNTAX__DYNAMICS_DIMENSION_HPP_

#include <scenario_runner/object.hpp>

namespace scenario_runner { inline namespace syntax
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
      : value { value }
    {}

    constexpr operator value_type() const noexcept
    {
      return value;
    }
  };

  template <typename... Ts>
  std::basic_istream<Ts...>& operator >>(std::basic_istream<Ts...>& is, DynamicsDimension& dimension)
  {
    std::string buffer {};

    is >> buffer;

    #define SUPPORTED(IDENTIFIER)                                              \
    if (buffer == #IDENTIFIER) do                                              \
    {                                                                          \
      dimension = DynamicsDimension::IDENTIFIER;                               \
      return is;                                                               \
    } while (false)

    SUPPORTED(rate);
    SUPPORTED(time);
    SUPPORTED(distance);

    #undef SUPPORTED

    #define UNSUPPORTED(IDENTIFIER)                                            \
    if (buffer == #IDENTIFIER) do                                              \
    {                                                                          \
      std::stringstream ss {};                                                 \
      ss << "given value \'" << buffer << "\' is valid OpenSCENARIO value of type DynamicsDimension, but it is not supported"; \
      throw ImplementationFault { ss.str() };                                  \
    } while (false)

    #undef UNSUPPORTED

    std::stringstream ss {};
    ss << "unexpected value \'" << buffer << "\' specified as type DynamicsDimension";
    throw SyntaxError { ss.str() };
  }

  template <typename... Ts>
  std::basic_ostream<Ts...>& operator <<(std::basic_ostream<Ts...>& os, const DynamicsDimension& dimension)
  {
    switch (dimension)
    {
      #define BOILERPLATE(NAME) case DynamicsDimension::NAME: return os << #NAME;

      BOILERPLATE(rate);
      BOILERPLATE(time);
      BOILERPLATE(distance);

      #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class DynamicsDimension holds unexpected value " << static_cast<DynamicsDimension::value_type>(dimension);
      throw ImplementationFault { ss.str() };
    }
  }
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__DYNAMICS_DIMENSION_HPP_
