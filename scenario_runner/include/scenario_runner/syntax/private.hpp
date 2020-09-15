#ifndef SCENARIO_RUNNER__SYNTAX__PRIVATE_HPP_
#define SCENARIO_RUNNER__SYNTAX__PRIVATE_HPP_

#include <scenario_runner/syntax/private_action.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== Private ==============================================================
 *
 * <xsd:complexType name="Private">
 *   <xsd:sequence>
 *     <xsd:element name="PrivateAction" type="PrivateAction" maxOccurs="unbounded"/>
 *   </xsd:sequence>
 *   <xsd:attribute name="entityRef" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Private
  : public std::vector<PrivateAction>
{
  Scope inner_scope;

  template<typename Node>
  explicit Private(const Node & node, Scope & outer_scope)
  : inner_scope{outer_scope}
  {
    inner_scope.actors.emplace_back(
      readAttribute<String>(node, inner_scope, "entityRef"));

    callWithElements(node, "PrivateAction", 1, unbounded, [&](auto && node)
      {
        emplace_back(node, inner_scope);
      });
  }

  auto evaluate()
  {
    for (auto && each : *this) {
      each.start();
    }

    return unspecified;
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__PRIVATE_HPP_
