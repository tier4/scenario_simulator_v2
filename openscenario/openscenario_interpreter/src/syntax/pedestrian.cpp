// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/pedestrian.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Pedestrian::Pedestrian(const pugi::xml_node & node, Scope & scope)
: Scope(readAttribute<String>("name", node, scope), scope),
  mass(readAttribute<Double>("mass", node, local())),
  model(readAttribute<String>("model", node, local())),
  model3d(readAttribute<String>("model3d", node, local(), "")),
  pedestrian_category(readAttribute<PedestrianCategory>("pedestrianCategory", node, local())),
  parameter_declarations(
    readElement<ParameterDeclarations>("ParameterDeclarations", node, local())),
  bounding_box(readElement<BoundingBox>("BoundingBox", node, local())),
  properties(readElement<Properties>("Properties", node, local()))
{
}

Pedestrian::operator traffic_simulator_msgs::msg::PedestrianParameters() const
{
  traffic_simulator_msgs::msg::PedestrianParameters parameter;
  {
    using namespace traffic_simulator_msgs;

    parameter.name = name;
    parameter.subtype = static_cast<msg::EntitySubtype>(pedestrian_category);
    parameter.bounding_box = static_cast<msg::BoundingBox>(bounding_box);
    parameter.obstacle_detect_mode =
      compatibility == Compatibility::legacy
        ? traffic_simulator_msgs::msg::PedestrianParameters::LEGACY
        : traffic_simulator_msgs::msg::PedestrianParameters::STANDARD;
  }

  return parameter;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
