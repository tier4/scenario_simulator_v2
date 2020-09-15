#ifndef SCENARIO_RUNNER__SYNTAX__WORLD_POSITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__WORLD_POSITION_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
// #include <quaternion_operation/quaternion_operation.h>

namespace scenario_runner { inline namespace syntax
{
  /* ==== WorldPosition ========================================================
   *
   * <xsd:complexType name="WorldPosition">
   *   <xsd:attribute name="x" type="Double" use="required"/>
   *   <xsd:attribute name="y" type="Double" use="required"/>
   *   <xsd:attribute name="z" type="Double" use="optional"/>
   *   <xsd:attribute name="h" type="Double" use="optional"/>
   *   <xsd:attribute name="p" type="Double" use="optional"/>
   *   <xsd:attribute name="r" type="Double" use="optional"/>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct WorldPosition
  {
    const Double x, y, z, h, p, r;

    template <typename Node, typename Scope>
    explicit WorldPosition(const Node& node, Scope& scope)
      : x { readAttribute<Double>(node, scope, "x") }
      , y { readAttribute<Double>(node, scope, "y") }
      , z { readAttribute<Double>(node, scope, "z", 0) }
      , h { readAttribute<Double>(node, scope, "h", 0) } // yaw
      , p { readAttribute<Double>(node, scope, "p", 0) }
      , r { readAttribute<Double>(node, scope, "r", 0) }
    {}

    operator geometry_msgs::msg::Pose() const
    {
      geometry_msgs::msg::Vector3 vector {};
      vector.x = r;
      vector.y = p;
      vector.z = h;

      geometry_msgs::msg::Point point {};
      point.x = x;
      point.y = y;
      point.z = z;

      geometry_msgs::msg::Pose pose {};
      pose.position = point;
      // pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(vector);

      return pose;
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__WORLD_POSITION_HPP_
