#ifndef SCENARIO_RUNNER__SYNTAX__FILE_HPP_
#define SCENARIO_RUNNER__SYNTAX__FILE_HPP_

#include <scenario_runner/reader/attribute.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== File =================================================================
 *
 * <xsd:complexType name="File">
 *   <xsd:attribute name="filepath" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct File
{
  const String filepath;

  explicit File()
  : filepath{"./"}
  {}

  template<typename Node, typename Scope>
  explicit File(const Node & node, Scope & outer_scope)
  : filepath{readAttribute<String>(node, outer_scope, "filepath")}
  {}
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__FILE_HPP_
