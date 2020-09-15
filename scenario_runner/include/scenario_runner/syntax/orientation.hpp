#ifndef SCENARIO_RUNNER__SYNTAX__ORIENTATION_HPP_
#define SCENARIO_RUNNER__SYNTAX__ORIENTATION_HPP_

#include <scenario_runner/reader/attribute.hpp>
#include <scenario_runner/syntax/reference_context.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== Orientation ==========================================================
 *
 * <xsd:complexType name="Orientation">
 *   <xsd:attribute name="type" type="ReferenceContext" use="optional"/>
 *   <xsd:attribute name="h" type="Double" use="optional"/>
 *   <xsd:attribute name="p" type="Double" use="optional"/>
 *   <xsd:attribute name="r" type="Double" use="optional"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Orientation
{
  const ReferenceContext type;

  const Double h, p, r;

  Orientation() = default;

  template<typename Node, typename Scope>
  explicit Orientation(const Node & node, Scope & scope)
  : type{readAttribute<ReferenceContext>(node, scope, "type", ReferenceContext::relative)},
    h{readAttribute<Double>(node, scope, "h", 0)},
    p{readAttribute<Double>(node, scope, "p", 0)},
    r{readAttribute<Double>(node, scope, "r", 0)}
  {}
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__ORIENTATION_HPP_
