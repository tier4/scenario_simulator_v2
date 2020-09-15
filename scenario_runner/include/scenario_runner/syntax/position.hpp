#ifndef SCENARIO_RUNNER__SYNTAX__POSITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__POSITION_HPP_

#include <scenario_runner/syntax/lane_position.hpp>
#include <scenario_runner/syntax/world_position.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== Position =============================================================
   *
   * <xsd:complexType name="Position">
   *   <xsd:choice>
   *     <xsd:element name="WorldPosition" type="WorldPosition"/>
   *     <xsd:element name="RelativeWorldPosition" type="RelativeWorldPosition"/>
   *     <xsd:element name="RelativeObjectPosition" type="RelativeObjectPosition"/>
   *     <xsd:element name="RoadPosition" type="RoadPosition"/>
   *     <xsd:element name="RelativeRoadPosition" type="RelativeRoadPosition"/>
   *     <xsd:element name="LanePosition" type="LanePosition"/>
   *     <xsd:element name="RelativeLanePosition" type="RelativeLanePosition"/>
   *     <xsd:element name="RoutePosition" type="RoutePosition"/>
   *   </xsd:choice>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct Position
    : public Object
  {
    template <typename Node, typename Scope>
    explicit Position(const Node& node, Scope& scope)
    {
      callWithElements(node, "WorldPosition", 0, 1, [&](auto&& node)
      {
        return rebind<WorldPosition>(node, scope);
      });

      callWithElements(node, "RelativeWorldPosition", 0, 1, THROW_UNSUPPORTED_ERROR(node));
      callWithElements(node, "RelativeObjectPosition", 0, 1, THROW_UNSUPPORTED_ERROR(node));
      callWithElements(node, "RoadPosition", 0, 1, THROW_UNSUPPORTED_ERROR(node));
      callWithElements(node, "RelativeRoadPosition", 0, 1, THROW_UNSUPPORTED_ERROR(node));

      callWithElements(node, "LanePosition", 0, 1, [&](auto&& node)
      {
        return rebind<LanePosition>(node, scope);
      });

      callWithElements(node, "RelativeLanePosition", 0, 1, THROW_UNSUPPORTED_ERROR(node));
      callWithElements(node, "RoutePosition", 0, 1, THROW_UNSUPPORTED_ERROR(node));
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__POSITION_HPP_
