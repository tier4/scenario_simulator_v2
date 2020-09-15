#ifndef SCENARIO_RUNNER__SYNTAX__ENTIRY_REF_HPP_
#define SCENARIO_RUNNER__SYNTAX__ENTIRY_REF_HPP_

#include <scenario_runner/reader/attribute.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== EntityRef ============================================================
   *
   * <xsd:complexType name="EntityRef">
   *   <xsd:attribute name="entityRef" type="String" use="required"/>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct EntityRef
    : public String
  {
    template <typename... Ts>
    explicit constexpr EntityRef(Ts&&... xs)
      : String { std::forward<decltype(xs)>(xs)... }
    {}

    template <typename Node, typename Scope>
    explicit EntityRef(const Node& node, Scope& scope)
      : String { readAttribute<String>(node, scope, "entityRef") }
    {}
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__ENTIRY_REF_HPP_
