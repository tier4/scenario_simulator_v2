#ifndef SCENARIO_RUNNER__SYNTAX__FILE_HEADER_HPP_
#define SCENARIO_RUNNER__SYNTAX__FILE_HEADER_HPP_

#include <scenario_runner/reader/attribute.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== FileHeader ===========================================================
 *
 * <xsd:complexType name="FileHeader">
 *   <xsd:attribute name="revMajor" type="UnsignedShort" use="required"/>
 *   <xsd:attribute name="revMinor" type="UnsignedShort" use="required"/>
 *   <xsd:attribute name="date" type="DateTime" use="required"/>
 *   <xsd:attribute name="description" type="String" use="required"/>
 *   <xsd:attribute name="author" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct FileHeader
{
  const UnsignedShort revMajor;
  const UnsignedShort revMinor;

  const String date;   // TODO type DateTime

  const String description;

  const String author;

  template<typename Node, typename Scope>
  explicit FileHeader(const Node & node, Scope & outer_scope)
  : revMajor{readAttribute<UnsignedShort>(node, outer_scope, "revMajor")},
    revMinor{readAttribute<UnsignedShort>(node, outer_scope, "revMinor")},
    date{readAttribute<String>(node, outer_scope, "date")},
    description{readAttribute<String>(node, outer_scope, "description")},
    author{readAttribute<String>(node, outer_scope, "author")}
  {}
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__FILE_HEADER_HPP_
