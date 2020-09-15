#ifndef SCENARIO_RUNNER__SYNTAX__BOUNDING_BOX_HPP_
#define SCENARIO_RUNNER__SYNTAX__BOUNDING_BOX_HPP_

#include <scenario_runner/syntax/center.hpp>
#include <scenario_runner/syntax/dimensions.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== BoundingBox ==========================================================
 *
 * <xsd:complexType name="BoundingBox">
 *   <xsd:all>
 *     <xsd:element name="Center" type="Center"/>
 *     <xsd:element name="Dimensions" type="Dimensions"/>
 *   </xsd:all>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct BoundingBox
{
  const Center center;

  const Dimensions dimensions;

  BoundingBox() = default;

  template<typename Node, typename Scope>
  explicit BoundingBox(const Node & node, Scope & scope)
  : center{readElement<Center>("Center", node, scope)},
    dimensions{readElement<Dimensions>("Dimensions", node, scope)}
  {}
};

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const BoundingBox & rhs)
{
  return os << (indent++) << blue << "<BoundingBox>\n" << reset <<
         rhs.center << "\n" <<
         rhs.dimensions << "\n" <<
         (--indent) << blue << "</BoundingBox>" << reset;
}
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__BOUNDING_BOX_HPP_
