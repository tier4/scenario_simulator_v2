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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__POISSON_DISTRIBUTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__POISSON_DISTRIBUTION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/range.hpp>
#include <random>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   PoissonDistribution (OpenSCENARIO XML 1.3.1)

   Poisson distribution which can be applied to a single parameter.

   <xsd:complexType name="PoissonDistribution">
     <xsd:sequence>
       <xsd:element name="Range" type="Range" minOccurs="0"/>
     </xsd:sequence>
     <xsd:attribute name="expectedValue" type="Double" use="required"/>
   </xsd:complexType>
*/
struct PoissonDistribution : public ComplexType, private Scope
{
  const Range range;

  const Double expected_value;

  std::poisson_distribution<> distribute;

  std::mt19937 random_engine;

  explicit PoissonDistribution(const pugi::xml_node &, Scope & scope);

  auto evaluate() -> Object;
};
}  // namespace syntax
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__POISSON_DISTRIBUTION_HPP_
