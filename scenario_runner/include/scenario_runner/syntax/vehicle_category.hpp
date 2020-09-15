#ifndef SCENARIO_RUNNER__SYNTAX__VEHICLE_CATEGORY_HPP_
#define SCENARIO_RUNNER__SYNTAX__VEHICLE_CATEGORY_HPP_

#include <scenario_runner/object.hpp>

namespace scenario_runner
{inline namespace syntax
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

  explicit VehicleCategory() = default;

  explicit constexpr VehicleCategory(value_type value)
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

    #define SUPPORTED(IDENTIFIER) \
  if (buffer == #IDENTIFIER) do \
    { \
      category.value = VehicleCategory::IDENTIFIER; \
      return is; \
    } while (false)

  SUPPORTED(bicycle);
  SUPPORTED(bus);
  SUPPORTED(car);
  SUPPORTED(motorbike);
  SUPPORTED(truck);

    #undef SUPPORTED

    #define UNSUPPORTED(IDENTIFIER) \
  if (buffer == #IDENTIFIER) do \
    { \
      std::stringstream ss {}; \
      ss << "given value \'" << buffer << \
        "\' is valid OpenSCENARIO value of type VehicleCategory, but it is not supported"; \
      throw ImplementationFault {ss.str()}; \
    } while (false)

  UNSUPPORTED(semitrailer);
  UNSUPPORTED(trailer);
  UNSUPPORTED(train);
  UNSUPPORTED(tram);
  UNSUPPORTED(van);

    #undef UNSUPPORTED

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
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__VEHICLE_CATEGORY_HPP_
