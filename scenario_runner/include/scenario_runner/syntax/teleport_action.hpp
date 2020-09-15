#ifndef SCENARIO_RUNNER__SYNTAX__TELEPORT_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__TELEPORT_ACTION_HPP_

#include <scenario_runner/syntax/position.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== TeleportAction =======================================================
 *
 * <xsd:complexType name="TeleportAction">
 *   <xsd:sequence>
 *     <xsd:element name="Position" type="Position"/>
 *   </xsd:sequence>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct TeleportAction
{
  Scope inner_scope;

  const Position position;

  template<typename Node>
  explicit TeleportAction(const Node & node, Scope & outer_scope)
  : inner_scope{outer_scope},
    position{readElement<Position>("Position", node, inner_scope)}
  {}

  void start() const
  {
    if (position.is<LanePosition>()) {
      // geometry_msgs::msg::Vector3 orientation {};
      //
      // switch (position.as<LanePosition>().orientation.type)
      // {
      // case ReferenceContext::relative:
      //   orientation.x = position.as<LanePosition>().orientation.h;
      //   orientation.y = position.as<LanePosition>().orientation.p;
      //   orientation.z = position.as<LanePosition>().orientation.r;
      //   break;
      //
      // case ReferenceContext::absolute:
      //   THROW(ImplementationFault);
      // }
      //
      // const simulation_controller::entity::EntityStatus status {
      //   inner_scope.connection->simulation->getCurrentTime(),
      //   Integer(position.as<LanePosition>().lane_id),
      //   position.as<LanePosition>().s,
      //   position.as<LanePosition>().offset,
      //   orientation,
      //   geometry_msgs::Twist {},
      //   geometry_msgs::Accel {}
      // };
      //
      // for (const auto& each : inner_scope.actors)
      // {
      //   inner_scope.connection->entity->setEntityStatus(each, status);
      // }
    } else {
      THROW(ImplementationFault);
    }
  }

  static constexpr auto accomplished() noexcept
  {
    return true;
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__TELEPORT_ACTION_HPP_
