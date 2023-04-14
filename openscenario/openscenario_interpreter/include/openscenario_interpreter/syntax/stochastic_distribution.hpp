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

#ifndef OPENSCENARIO_INTERPRETER__STOCHASTIC_DISTRIBUTION_HPP_
#define OPENSCENARIO_INTERPRETER__STOCHASTIC_DISTRIBUTION_HPP_

#include <openscenario_interpreter/parameter_distribution.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/stochastic_distribution_type.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- StochasticDistribution 1.2 ---------------------------------------------
 *
 *  <xsd:complexType name="StochasticDistribution">
 *    <xsd:sequence>
 *      <xsd:group ref="StochasticDistributionType"/>
 *    </xsd:sequence>
 *    <xsd:attribute name="parameterName" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct StochasticDistribution : public StochasticDistributionType,
                                public ParameterDistributionContainer
{
  const String parameter_name;

  explicit StochasticDistribution(const pugi::xml_node &, Scope & scope);

  auto derive() -> ParameterDistribution override;

  auto derive(size_t local_index, size_t local_size, size_t global_index, size_t global_size)
    -> ParameterList override;
};
}  // namespace syntax
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__STOCHASTIC_DISTRIBUTION_HPP_
