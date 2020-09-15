#ifndef SCENARIO_RUNNER__SYNTAX__ROAD_NETWORK_HPP_
#define SCENARIO_RUNNER__SYNTAX__ROAD_NETWORK_HPP_

#include <scenario_runner/syntax/file.hpp>
#include <scenario_runner/syntax/traffic_signals.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== RoadNetwork ==========================================================
   *
   * <xsd:complexType name="RoadNetwork">
   *   <xsd:sequence>
   *     <xsd:element name="LogicFile" type="File" minOccurs="0"/>
   *     <xsd:element name="SceneGraphFile" type="File" minOccurs="0"/>
   *     <xsd:element name="TrafficSignals" minOccurs="0" type="TrafficSignals"/>
   *   </xsd:sequence>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct RoadNetwork
    : public Sequence
  {
    const File logic_file;

    const File scene_graph_file;

    const TrafficSignals traffic_signals;

    template <typename Node, typename Scope>
    explicit RoadNetwork(const Node& node, Scope& outer_scope)
      : logic_file       { readElement<File>          (     "LogicFile", node, outer_scope) }
      , scene_graph_file { readElement<File>          ("SceneGraphFile", node, outer_scope) }
      , traffic_signals  { readElement<TrafficSignals>("TrafficSignals", node, outer_scope) }
    {}
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__ROAD_NETWORK_HPP_
