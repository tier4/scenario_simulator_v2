#ifndef SCENARIO_RUNNER__SYNTAX__ASSIGN_ROUTE_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__ASSIGN_ROUTE_ACTION_HPP_

#include <scenario_runner/syntax/route.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== AssignRouteAction ====================================================
   *
   * <xsd:complexType name="AssignRouteAction">
   *   <xsd:choice>
   *     <xsd:element name="Route" type="Route"/>
   *     <xsd:element name="CatalogReference" type="CatalogReference"/>
   *   </xsd:choice>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct AssignRouteAction
    : public Object
  {
    template <typename Node, typename Scope>
    explicit AssignRouteAction(const Node& node, Scope& scope)
    {
      callWithElements(node, "Route", 0, 1, [&](auto&& node)
      {
        return rebind<Route>(node, scope);
      });

      callWithElements(node, "CatalogReference", 0, 1, THROW_UNSUPPORTED_ERROR(node));
    }

    auto evaluate() const noexcept
    {
      return unspecified;
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__ASSIGN_ROUTE_ACTION_HPP_
