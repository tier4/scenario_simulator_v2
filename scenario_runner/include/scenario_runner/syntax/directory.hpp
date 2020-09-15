#ifndef SCENARIO_RUNNER__SYNTAX__DIRECTORY_HPP_
#define SCENARIO_RUNNER__SYNTAX__DIRECTORY_HPP_

#include <scenario_runner/scope.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== Directory ============================================================
   *
   * <xsd:complexType name="Directory">
   *   <xsd:attribute name="path" type="String" use="required"/>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct Directory
  {
    const String path;

    template <typename Node, typename Scope>
    explicit Directory(const Node& node, Scope& outer_scope)
      : path { readAttribute<String>(node, outer_scope, "path") }
    {}
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__DIRECTORY_HPP_
