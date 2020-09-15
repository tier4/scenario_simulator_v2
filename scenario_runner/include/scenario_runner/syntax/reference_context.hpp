#ifndef SCENARIO_RUNNER__SYNTAX__REFERENCE_CONTEXT_HPP_
#define SCENARIO_RUNNER__SYNTAX__REFERENCE_CONTEXT_HPP_

#include <scenario_runner/object.hpp>

namespace scenario_runner { inline namespace syntax
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
      : value { value }
    {}

    constexpr operator value_type() const noexcept
    {
      return value;
    }
  };

  template <typename... Ts>
  std::basic_istream<Ts...>& operator >>(std::basic_istream<Ts...>& is, ReferenceContext& context)
  {
    std::string buffer {};

    is >> buffer;

    #define BOILERPLATE(IDENTIFIER)                                            \
    if (buffer == #IDENTIFIER) do                                              \
    {                                                                          \
      context.value = ReferenceContext::IDENTIFIER;                            \
      return is;                                                               \
    } while (false)

    BOILERPLATE(relative);

    #undef BOILERPLATE

    #define BOILERPLATE(IDENTIFIER)                                            \
    if (buffer == #IDENTIFIER) do                                              \
    {                                                                          \
      std::stringstream ss {};                                                 \
      ss << "given value \'" << buffer << "\' is valid OpenSCENARIO value of type ReferenceContext, but it is not supported"; \
      throw ImplementationFault { ss.str() };                                  \
    } while (false)

    BOILERPLATE(absolute);

    #undef BOILERPLATE

    std::stringstream ss {};
    ss << "unexpected value \'" << buffer << "\' specified as type ReferenceContext";
    throw SyntaxError { ss.str() };
  }

  template <typename... Ts>
  std::basic_ostream<Ts...>& operator <<(std::basic_ostream<Ts...>& os, const ReferenceContext& context)
  {
    switch (context)
    {
      #define BOILERPLATE(IDENTIFIER) case ReferenceContext::IDENTIFIER: return os << #IDENTIFIER;

      BOILERPLATE(absolute);
      BOILERPLATE(relative);

      #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class ReferenceContext holds unexpected value " << static_cast<ReferenceContext::value_type>(context.value);
      throw ImplementationFault { ss.str() };
    }
  }
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__REFERENCE_CONTEXT_HPP_
