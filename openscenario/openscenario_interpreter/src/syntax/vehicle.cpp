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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/vehicle.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Vehicle::Vehicle(const pugi::xml_node & node, Scope & scope)
: Scope(readAttribute<String>("name", node, scope), scope),
  vehicle_category(readAttribute<VehicleCategory>("vehicleCategory", node, local())),
  parameter_declarations(
    readElement<ParameterDeclarations>("ParameterDeclarations", node, local())),
  bounding_box(readElement<BoundingBox>("BoundingBox", node, local())),
  performance(readElement<Performance>("Performance", node, local())),
  axles(readElement<Axles>("Axles", node, local())),
  properties(readElement<Properties>("Properties", node, local()))
{
}

Vehicle::operator traffic_simulator_msgs::msg::VehicleParameters() const
{
  traffic_simulator_msgs::msg::VehicleParameters parameter;
  {
    using namespace traffic_simulator_msgs;

    parameter.name = name;
    parameter.subtype = static_cast<msg::EntitySubtype>(vehicle_category);
    parameter.bounding_box = static_cast<msg::BoundingBox>(bounding_box);
    parameter.performance = static_cast<msg::Performance>(performance);
    parameter.axles = static_cast<msg::Axles>(axles);
  }

  return parameter;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
