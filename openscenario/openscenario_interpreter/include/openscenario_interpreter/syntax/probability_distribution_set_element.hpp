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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PROBABILITY_DISTRIBUTION_SET_ELEMENT_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PROBABILITY_DISTRIBUTION_SET_ELEMENT_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/string.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   ProbabilityDistributionSetElement (OpenSCENARIO XML 1.3.1)

   Indicates a value and probability in a stochastic distribution.

   <xsd:complexType name="ProbabilityDistributionSetElement">
     <xsd:attribute name="value" type="String" use="required"/>
     <xsd:attribute name="weight" type="Double" use="required"/>
   </xsd:complexType>
*/
struct ProbabilityDistributionSetElement : public ComplexType
{
  const String value;

  const Double weight;

  explicit ProbabilityDistributionSetElement(const pugi::xml_node &, Scope & scope);
};
}  // namespace syntax
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PROBABILITY_DISTRIBUTION_SET_ELEMENT_HPP_
