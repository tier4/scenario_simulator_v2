#ifndef SCENARIO_RUNNER__SYNTAX__RELATIVE_DISTANCE_TYPE_HPP_
#define SCENARIO_RUNNER__SYNTAX__RELATIVE_DISTANCE_TYPE_HPP_

namespace scenario_runner
{inline namespace syntax
{
/* ==== RelativeDistanceType =================================================
 *
 * <xsd:simpleType name="RelativeDistanceType">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="longitudinal"/>
 *         <xsd:enumeration value="lateral"/>
 *         <xsd:enumeration value="cartesianDistance"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * ======================================================================== */
struct RelativeDistanceType
{
  enum value_type
  {
    longitudinal,
    lateral,
    cartesianDistance,
  }
  value;

  explicit RelativeDistanceType() = default;

  explicit RelativeDistanceType(value_type value)
  : value{value}
  {}

  operator value_type() const noexcept
  {
    return value;
  }
};

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(std::basic_istream<Ts...> & is, RelativeDistanceType & type)
{
  std::string buffer {};

  is >> buffer;

    #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) do \
    { \
      type.value = RelativeDistanceType::IDENTIFIER; \
      return is; \
    } while (false)

  BOILERPLATE(longitudinal);
  BOILERPLATE(lateral);
  BOILERPLATE(cartesianDistance);

    #undef BOILERPLATE

  std::stringstream ss {};
  ss << "unexpected value \'" << buffer << "\' specified as type RelativeDistanceType";
  throw SyntaxError {ss.str()};
}

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(
  std::basic_ostream<Ts...> & os,
  const RelativeDistanceType & type)
{
  switch (type) {
      #define BOILERPLATE(IDENTIFIER) case RelativeDistanceType::IDENTIFIER: return os << \
           #IDENTIFIER;

    BOILERPLATE(longitudinal);
    BOILERPLATE(lateral);
    BOILERPLATE(cartesianDistance);

      #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class RelativeDistanceType holds unexpected value " <<
        static_cast<RelativeDistanceType::value_type>(type.value);
      throw ImplementationFault {ss.str()};
  }
}
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__RELATIVE_DISTANCE_TYPE_HPP_
