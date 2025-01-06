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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__DETERMINISTIC_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__DETERMINISTIC_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/deterministic_parameter_distribution.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   Deterministic (OpenSCENARIO XML 1.3.1)

   Top level container containing all deterministic distribution elements.

   <xsd:complexType name="Deterministic">
     <xsd:sequence>
       <xsd:group ref="DeterministicParameterDistribution" minOccurs="0" maxOccurs="unbounded"/>
     </xsd:sequence>
   </xsd:complexType>
*/
struct Deterministic
{
  const std::list<DeterministicParameterDistribution> deterministic_parameter_distributions;

  explicit Deterministic(const pugi::xml_node &, Scope & scope);
};
}  // namespace syntax
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__DETERMINISTIC_HPP_
