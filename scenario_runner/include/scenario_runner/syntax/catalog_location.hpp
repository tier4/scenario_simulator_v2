#ifndef SCENARIO_RUNNER__SYNTAX__CATALOG_LOCATION_HPP_
#define SCENARIO_RUNNER__SYNTAX__CATALOG_LOCATION_HPP_

#include <scenario_runner/reader/element.hpp>
#include <scenario_runner/syntax/directory.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== CatalogLocation ======================================================
   *
   * <xsd:complexType name="VehicleCatalogLocation">
   *   <xsd:all>
   *     <xsd:element name="Directory" type="Directory"/>
   *   </xsd:all>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct CatalogLocation
  {
    const Directory directory;

    template <typename Node, typename Scope>
    explicit CatalogLocation(const Node& node, Scope& outer_scope)
      : directory { readElement<Directory>("Directory", node, outer_scope) }
    {}
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__CATALOG_LOCATION_HPP_
