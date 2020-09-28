// Copyright 2015-2020 Autoware Foundation. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SCENARIO_RUNNER__SYNTAX__ROAD_NETWORK_HPP_
#define SCENARIO_RUNNER__SYNTAX__ROAD_NETWORK_HPP_

#include <scenario_runner/syntax/file.hpp>
#include <scenario_runner/syntax/traffic_signals.hpp>

namespace scenario_runner
{
inline namespace syntax
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
{
  const File logic_file;

  const File scene_graph_file;

  const TrafficSignals traffic_signals;

  template<typename Node, typename Scope>
  explicit RoadNetwork(const Node & node, Scope & outer_scope)
  : logic_file{readElement<File>("LogicFile", node, outer_scope)},
    scene_graph_file{readElement<File>("SceneGraphFile", node, outer_scope)},
    traffic_signals{readElement<TrafficSignals>("TrafficSignals", node, outer_scope)}
  {}
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__ROAD_NETWORK_HPP_
