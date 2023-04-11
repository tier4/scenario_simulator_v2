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

#ifndef OPENSCENARIO_INTERPRETER__STOCHASTIC_HPP_
#define OPENSCENARIO_INTERPRETER__STOCHASTIC_HPP_

#include <openscenario_interpreter/parameter_distribution.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/stochastic_distribution.hpp>
#include <openscenario_interpreter/syntax/unsigned_integer.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Stochastic 1.2 ---------------------------------------------------------
 *
 *  <xsd:complexType name="Stochastic">
 *    <xsd:sequence>
 *      <xsd:element name="StochasticDistribution"
 *        type="StochasticDistribution" maxOccurs="unbounded"/>
 *    </xsd:sequence>
 *    <xsd:attribute name="numberOfTestRuns" type="UnsignedInt" use="required"/>
 *    <xsd:attribute name="randomSeed" type="Double"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */

struct Stochastic : public ComplexType, public ParameterDistributionContainer
{
  const UnsignedInt number_of_test_runs;

  const Double random_seed;

  StochasticDistribution stochastic_distribution;

  explicit Stochastic(const pugi::xml_node &, Scope & scope);

  auto derive() -> ParameterDistribution override
  {
    ParameterDistribution distribution;
    for (size_t i = 0; i < number_of_test_runs; i++) {
      auto derived = stochastic_distribution.derive();
      distribution.insert(distribution.end(), derived.begin(), derived.end());
    }
    return distribution;
  }

  auto getNumberOfDeriveScenarios() const -> size_t override { return number_of_test_runs; }
};
}  // namespace syntax
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__STOCHASTIC_HPP_
