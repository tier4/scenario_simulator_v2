// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/road_network.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
RoadNetwork::RoadNetwork(const pugi::xml_node & node, Scope & scope)
: logic_file(readElement<File>("LogicFile", node, scope)),
  scene_graph_file(readElement<File>("SceneGraphFile", node, scope)),
  traffic_signals(readElement<TrafficSignals>("TrafficSignals", node, scope))
{
}

auto RoadNetwork::evaluate() -> Object { return traffic_signals.evaluate(); }
}  // namespace syntax
}  // namespace openscenario_interpreter
