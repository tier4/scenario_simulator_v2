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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PROBABILITY_DISTRIBUTION_SET_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PROBABILITY_DISTRIBUTION_SET_HPP_

#include <openscenario_interpreter/syntax/probability_distribution_set_element.hpp>
#include <random>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   ProbabilityDistributionSet (OpenSCENARIO XML 1.3.1)

   Container for a set of single values with a defined probability.

   <xsd:complexType name="ProbabilityDistributionSet">
     <xsd:sequence>
       <xsd:element name="Element" type="ProbabilityDistributionSetElement" maxOccurs="unbounded"/>
     </xsd:sequence>
   </xsd:complexType>
*/
struct ProbabilityDistributionSet : public ComplexType, private Scope
{
  const std::vector<ProbabilityDistributionSetElement> elements;

  struct ProbabilityDistributionSetAdaptor
  {
    explicit ProbabilityDistributionSetAdaptor(
      const std::vector<ProbabilityDistributionSetElement> & elements)
    {
      for (const auto & element : elements) {
        probabilities.emplace_back(element.weight);
        values.emplace_back(element.value);
      }
    }
    std::vector<double> probabilities;
    std::vector<String> values;
  } adaptor;

  std::discrete_distribution<std::size_t> distribute;

  std::mt19937 random_engine;

  explicit ProbabilityDistributionSet(const pugi::xml_node &, Scope & scope);

  auto evaluate() -> Object;
};
}  // namespace syntax
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PROBABILITY_DISTRIBUTION_SET_HPP_
