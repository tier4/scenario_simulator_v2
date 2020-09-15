#ifndef SCENARIO_RUNNER__SYNTAX__LANE_POSITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__LANE_POSITION_HPP_

#include <scenario_runner/syntax/orientation.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== LanePosition =========================================================
 *
 * <xsd:complexType name="LanePosition">
 *   <xsd:all>
 *     <xsd:element name="Orientation" type="Orientation" minOccurs="0"/>
 *   </xsd:all>
 *   <xsd:attribute name="roadId" type="String" use="required"/>
 *   <xsd:attribute name="laneId" type="String" use="required"/>
 *   <xsd:attribute name="offset" type="Double" use="optional"/>
 *   <xsd:attribute name="s" type="Double" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct LanePosition
{
  const String road_id, lane_id;

  const Double offset, s;

  const Orientation orientation;

  template<typename Node, typename Scope>
  explicit LanePosition(const Node & node, Scope & scope)
  : road_id{readAttribute<String>(node, scope, "roadId", "")},
    lane_id{readAttribute<String>(node, scope, "laneId")},
    offset{readAttribute<Double>(node, scope, "offset", 0)},
    s{readAttribute<Double>(node, scope, "s")},
    orientation{readElement<Orientation>("Orientation", node, scope)}
  {}
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__LANE_POSITION_HPP_
