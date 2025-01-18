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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_ASSIGNMENT_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_ASSIGNMENT_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   ParameterAssignment (OpenSCENARIO XML 1.3.1)

   Assignment of a value to a named parameter. It is either used when importing
   types of vehicles, controllers etc. from a catalog during startup of a simulator.
   It is also used when generating concrete scenarios from logical scenarios
   with ParameterValueSets during runtime of a scenario generator.

   <xsd:complexType name="ParameterAssignment">
     <xsd:attribute name="parameterRef" type="String" use="required"/>
     <xsd:attribute name="value" type="String" use="required"/>
   </xsd:complexType>
*/
struct ParameterAssignment
{
  explicit ParameterAssignment(const pugi::xml_node & node, Scope & scope)
  : parameterRef(readAttribute<std::string>("parameterRef", node, scope)),
    value(readAttribute<std::string>("value", node, scope))
  {
    scope.insert(parameterRef, make<std::string>(value));
  }

  const std::string parameterRef;
  const std::string value;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_ASSIGNMENT_HPP_
