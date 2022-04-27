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
#include <openscenario_interpreter/syntax/misc_object.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
MiscObject::MiscObject(const pugi::xml_node & node, Scope & scope)
: Scope(readAttribute<String>("name", node, scope), scope),
  mass(readAttribute<Double>("mass", node, local())),
  misc_object_category(readAttribute<MiscObjectCategory>("miscObjectCategory", node, local())),
  parameter_declarations(
    readElement<ParameterDeclarations>("ParameterDeclarations", node, local())),
  bounding_box(readElement<BoundingBox>("BoundingBox", node, local())),
  properties(readElement<Properties>("Properties", node, local()))
{
}

MiscObject::operator traffic_simulator_msgs::msg::MiscObjectParameters() const
{
  traffic_simulator_msgs::msg::MiscObjectParameters misc_object_parameters;
  {
    using namespace traffic_simulator_msgs;

    misc_object_parameters.name = name;
    misc_object_parameters.subtype = static_cast<msg::EntitySubtype>(misc_object_category);
    misc_object_parameters.bounding_box = static_cast<msg::BoundingBox>(bounding_box);
  }

  return misc_object_parameters;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
