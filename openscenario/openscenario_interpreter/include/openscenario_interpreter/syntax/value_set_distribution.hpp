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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__VALUE_SET_DISTRIBUTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__VALUE_SET_DISTRIBUTION_HPP_

#include <openscenario_interpreter/syntax/file.hpp>
#include <openscenario_interpreter/syntax/parameter_value_set.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   ValueSetDistribution (OpenSCENARIO XML 1.3.1)

   Deterministic multi-parameter distribution, where one or multiple sets of parameter values can be defined.

   <xsd:complexType name="ValueSetDistribution">
     <xsd:sequence>
       <xsd:element name="ParameterValueSet" type="ParameterValueSet" maxOccurs="unbounded"/>
     </xsd:sequence>
   </xsd:complexType>
*/
struct ValueSetDistribution : public Scope
{
  const std::list<ParameterValueSet> parameter_value_sets;

  explicit ValueSetDistribution(const pugi::xml_node &, Scope & scope);

  // TODO: implement evaluate()
};
}  // namespace syntax
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__VALUE_SET_DISTRIBUTION_HPP_
