#ifndef SCENARIO_RUNNER__SYNTAX__CENTER_HPP_
#define SCENARIO_RUNNER__SYNTAX__CENTER_HPP_

#include <scenario_runner/reader/attribute.hpp>
#include <scenario_runner/reader/element.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== Center ===============================================================
   *
   * <xsd:complexType name="Center">
   *   <xsd:attribute name="x" type="Double" use="required"/>
   *   <xsd:attribute name="y" type="Double" use="required"/>
   *   <xsd:attribute name="z" type="Double" use="required"/>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct Center
  {
    const Double x, y, z;

    Center() = default;

    template <typename Node, typename Scope>
    explicit Center(const Node& node, Scope& scope)
      : x { readAttribute<Double>(node, scope, "x") }
      , y { readAttribute<Double>(node, scope, "y") }
      , z { readAttribute<Double>(node, scope, "z") }
    {}
  };

  template <typename... Ts>
  std::basic_ostream<Ts...>& operator <<(std::basic_ostream<Ts...>& os, const Center& rhs)
  {
    return os << indent << blue << "<Center" << " " << highlight("x", rhs.x)
                                             << " " << highlight("y", rhs.y)
                                             << " " << highlight("z", rhs.z) << blue << "/>" << reset;
  }
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__CENTER_HPP_
