// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ROAD_NETWORK_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ROAD_NETWORK_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/file.hpp>
#include <openscenario_interpreter/syntax/traffic_signals.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- RoadNetwork ------------------------------------------------------------
 *
 *  <xsd:complexType name="RoadNetwork">
 *    <xsd:sequence>
 *      <xsd:element name="LogicFile" type="File" minOccurs="0"/>
 *      <xsd:element name="SceneGraphFile" type="File" minOccurs="0"/>
 *      <xsd:element name="TrafficSignals" minOccurs="0" type="TrafficSignals"/>
 *    </xsd:sequence>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct RoadNetwork
{
  // File path of the road network file (e.g. an ASAM OpenDRIVE file).
  const File logic_file;

  // File path of a 3D model representing the virtual environment. This may be used for visual representation (rendering).
  const File scene_graph_file;

  // Name references and description of dynamic behavior for traffic signals defined in the road network file.
  TrafficSignals traffic_signals;

  explicit RoadNetwork(const pugi::xml_node &, Scope &);

  auto evaluate() -> Object;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ROAD_NETWORK_HPP_
