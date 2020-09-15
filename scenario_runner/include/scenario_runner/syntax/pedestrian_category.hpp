#ifndef SCENARIO_RUNNER__SYNTAX__PEDESTRIAN_CATEGORY_HPP_
#define SCENARIO_RUNNER__SYNTAX__PEDESTRIAN_CATEGORY_HPP_

#include <scenario_runner/object.hpp>

namespace scenario_runner { inline namespace syntax
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
      : value { value }
    {}

    constexpr operator value_type() const noexcept
    {
      return value;
    }
  };

  template <typename... Ts>
  std::basic_istream<Ts...>& operator >>(std::basic_istream<Ts...>& is, PedestrianCategory& category)
  {
    std::string buffer {};

    is >> buffer;

    #define SUPPORTED(IDENTIFIER)                                              \
    if (buffer == #IDENTIFIER) do                                              \
    {                                                                          \
      category.value = PedestrianCategory::IDENTIFIER;                         \
      return is;                                                               \
    } while (false)

    SUPPORTED(pedestrian);

    #undef SUPPORTED

    #define UNSUPPORTED(IDENTIFIER)                                            \
    if (buffer == #IDENTIFIER) do                                              \
    {                                                                          \
      std::stringstream ss {};                                                 \
      ss << "given value \'" << buffer << "\' is valid OpenSCENARIO value of type PedestrianCategory, but it is not supported"; \
      throw ImplementationFault { ss.str() };                                  \
    } while (false)

    UNSUPPORTED(wheelchair);
    UNSUPPORTED(animal);

    #undef UNSUPPORTED

    std::stringstream ss {};
    ss << "unexpected value \'" << buffer << "\' specified as type PedestrianCategory";
    throw SyntaxError { ss.str() };
  }

  template <typename... Ts>
  std::basic_ostream<Ts...>& operator <<(std::basic_ostream<Ts...>& os, const PedestrianCategory& category)
  {
    switch (category)
    {
      #define BOILERPLATE(NAME) case PedestrianCategory::NAME: return os << #NAME;

      BOILERPLATE(pedestrian);
      BOILERPLATE(wheelchair);
      BOILERPLATE(animal);

      #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class PedestrianCategory holds unexpected value " << static_cast<PedestrianCategory::value_type>(category);
      throw ImplementationFault { ss.str() };
    }
  }
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__PEDESTRIAN_CATEGORY_HPP_
