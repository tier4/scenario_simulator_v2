#ifndef SCENARIO_RUNNER__SYNTAX__HOGE_HPP_
#define SCENARIO_RUNNER__SYNTAX__HOGE_HPP_

namespace scenario_runner
{inline namespace syntax
{
/* ==== ObjectType ===========================================================
 *
 * <xsd:simpleType name="ObjectType">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="pedestrian"/>
 *         <xsd:enumeration value="vehicle"/>
 *         <xsd:enumeration value="miscellaneous"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * ======================================================================== */
struct ObjectType
{};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__HOGE_HPP_
