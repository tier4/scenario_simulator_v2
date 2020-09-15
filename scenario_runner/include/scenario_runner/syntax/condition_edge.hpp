#ifndef SCENARIO_RUNNER__SYNTAX__CONDITION_EDGE_HPP_
#define SCENARIO_RUNNER__SYNTAX__CONDITION_EDGE_HPP_

#include <scenario_runner/reader/attribute.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== ConditionEdge ============================================================
 *
 * <xsd:simpleType name="ConditionEdge">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="rising"/>
 *         <xsd:enumeration value="falling"/>
 *         <xsd:enumeration value="risingOrFalling"/>
 *         <xsd:enumeration value="none"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * ======================================================================== */
struct ConditionEdge
{
  enum value_type
  {
    /* ---- Rising -----------------------------------------------------------
     *
     * A condition defined with a rising edge shall return true at discrete
     * time t if its logical expression is true at discrete time t and its
     * logical expression was false at discrete time t-ts, where ts is the
     * simulation sampling time.
     *
     * -------------------------------------------------------------------- */
    rising,

    /* ---- Falling ----------------------------------------------------------
     *
     * A condition defined with a falling edge shall return true at discrete
     * time t if its logical expression is false at discrete time t and its
     * logical expression was true at discrete time t-ts, where ts is the
     * simulation sampling time.
     *
     * -------------------------------------------------------------------- */
    falling,

    /* ---- Rising or Falling ------------------------------------------------
     *
     * A condition defined with a 'risingOrFalling' edge shall return true at
     * discrete time t if its logical expression is true at discrete time t
     * and its logical expression was false at discrete time t-ts OR if its
     * logical expression is false at discrete time t and its logical
     * expression was true at discrete time t-ts. ts is the simulation
     * sampling time.
     *
     * -------------------------------------------------------------------- */
    risingOrFalling,

    /* ---- None -------------------------------------------------------------
     *
     * A condition defined with a 'none' edge shall return true at discrete
     * time t if its logical expression is true at discrete time t.
     *
     * -------------------------------------------------------------------- */
    none,
  }
  value;

  explicit ConditionEdge() = default;

  explicit ConditionEdge(value_type value)
  : value{value}
  {}

  operator value_type() const noexcept
  {
    return value;
  }
};

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(std::basic_istream<Ts...> & is, ConditionEdge & edge)
{
  std::string buffer {};

  is >> buffer;

    #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) do \
    { \
      edge.value = ConditionEdge::IDENTIFIER; \
      return is; \
    } while (false)

  BOILERPLATE(rising);
  BOILERPLATE(falling);
  BOILERPLATE(risingOrFalling);
  BOILERPLATE(none);

    #undef BOILERPLATE

  std::stringstream ss {};
  ss << "unexpected value \'" << buffer << "\' specified as type ConditionEdge";
  throw SyntaxError {ss.str()};
}

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const ConditionEdge & edge)
{
  switch (edge) {
      #define BOILERPLATE(IDENTIFIER) case ConditionEdge::IDENTIFIER: return os << #IDENTIFIER;

    BOILERPLATE(rising);
    BOILERPLATE(falling);
    BOILERPLATE(risingOrFalling);
    BOILERPLATE(none);

      #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class ConditionEdge holds unexpected value " <<
        static_cast<ConditionEdge::value_type>(edge.value);
      throw ImplementationFault {ss.str()};
  }
}
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__CONDITION_EDGE_HPP_
