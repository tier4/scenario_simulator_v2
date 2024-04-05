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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_ASSIGNMENTS_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_ASSIGNMENTS_HPP_

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/parameter_assignment.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ParameterAssignment--------------------------------------------------
 *
 * <xsd:complexType name="ParameterAssignment">
 *   <xsd:attribute name="parameterRef" type="String" use="required"/>
 *   <xsd:attribute name="value" type="String" use="required"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ParameterAssignments : std::list<ParameterAssignment>
{
  ParameterAssignments() = default;

  explicit ParameterAssignments(const pugi::xml_node & node, Scope & scope)
  : std::list<ParameterAssignment>(
      readElements<ParameterAssignment, 0>("ParameterAssignment", node, scope))
  {
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_ASSIGNMENTS_HPP_
