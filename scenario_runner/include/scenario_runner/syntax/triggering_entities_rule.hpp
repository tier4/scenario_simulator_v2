#ifndef SCENARIO_RUNNER__SYNTAX__TRIGGERING_ENTITIES_RULE_HPP_
#define SCENARIO_RUNNER__SYNTAX__TRIGGERING_ENTITIES_RULE_HPP_

#include <scenario_runner/object.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== TriggeringEntitiesRule ===============================================
 *
 * <xsd:simpleType name="TriggeringEntitiesRule">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="any"/>
 *         <xsd:enumeration value="all"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * ======================================================================== */
struct TriggeringEntitiesRule
{
  enum value_type
  {
    all, any, none,
  } value;

  explicit constexpr TriggeringEntitiesRule(value_type value = {})
  : value{value}
  {}

  constexpr operator value_type() const noexcept
  {
    return value;
  }

  template<typename ... Ts>
  constexpr decltype(auto) operator()(Ts && ... xs) const
  {
    switch (value) {
      case all:
        return std::all_of(std::forward<decltype(xs)>(xs)...);

      case any:
        return std::any_of(std::forward<decltype(xs)>(xs)...);

      case none:
        return std::none_of(std::forward<decltype(xs)>(xs)...);
    }
  }
};

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(
  std::basic_istream<Ts...> & is,
  TriggeringEntitiesRule & rule)
{
  std::string buffer {};

  is >> buffer;

    #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) do \
    { \
      rule.value = TriggeringEntitiesRule::IDENTIFIER; \
      return is; \
    } while (false)

  BOILERPLATE(all);
  BOILERPLATE(any);

    #undef BOILERPLATE

  std::stringstream ss {};
  ss << "unexpected value \'" << buffer << "\' specified as type TriggeringEntitiesRule";
  throw SyntaxError {ss.str()};
}

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(
  std::basic_ostream<Ts...> & os,
  const TriggeringEntitiesRule & rule)
{
  switch (rule) {
      #define BOILERPLATE(IDENTIFIER) case TriggeringEntitiesRule::IDENTIFIER: return os << \
           #IDENTIFIER;

    BOILERPLATE(all);
    BOILERPLATE(any);

      #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class TriggeringEntitiesRule holds unexpected value " <<
        static_cast<TriggeringEntitiesRule::value_type>(rule.value);
      throw ImplementationFault {ss.str()};
  }
}
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__TRIGGERING_ENTITIES_RULE_HPP_
