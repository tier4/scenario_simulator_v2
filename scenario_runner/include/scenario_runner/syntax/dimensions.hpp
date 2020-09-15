#ifndef SCENARIO_RUNNER__SYNTAX__DIMENSIONS_HPP_
#define SCENARIO_RUNNER__SYNTAX__DIMENSIONS_HPP_

#include <scenario_runner/reader/attribute.hpp>
#include <scenario_runner/reader/element.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== Dimensions ===========================================================
   *
   * <xsd:complexType name="Dimensionss">
   *   <xsd:attribute name="width" type="Double" use="required"/>
   *   <xsd:attribute name="length" type="Double" use="required"/>
   *   <xsd:attribute name="height" type="Double" use="required"/>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct Dimensions
  {
    const Double width, length, height;

    Dimensions() = default;

    template <typename Node, typename Scope>
    explicit Dimensions(const Node& node, Scope& scope)
      : width  { readAttribute<Double>(node, scope, "width") }
      , length { readAttribute<Double>(node, scope, "length") }
      , height { readAttribute<Double>(node, scope, "height") }
    {}
  };

  template <typename... Ts>
  std::basic_ostream<Ts...>& operator <<(std::basic_ostream<Ts...>& os, const Dimensions& rhs)
  {
    return os << indent << blue << "<Dimensions" << " " << highlight("width",  rhs.width)
                                                 << " " << highlight("length", rhs.length)
                                                 << " " << highlight("height", rhs.height) << blue << "/>" << reset;
  }
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__DIMENSIONS_HPP_
