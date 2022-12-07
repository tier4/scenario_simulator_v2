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

#ifndef OPENSCENARIO_INTERPRETER__UNIFORM_DISTRIBUTION_HPP_
#define OPENSCENARIO_INTERPRETER__UNIFORM_DISTRIBUTION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/range.hpp>
#include <openscenario_interpreter/utility/distribution.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- UniformDistribution 1.2 ------------------------------------------------
 *
 *  <xsd:complexType name="UniformDistribution">
 *    <xsd:sequence>
 *      <xsd:element name="Range" type="Range"/>
 *    </xsd:sequence>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct UniformDistribution : public ComplexType
{
  const Range range;

  StochasticDistributionClass<std::uniform_real_distribution<Double::value_type>> distribution;

  explicit UniformDistribution(const pugi::xml_node &, Scope & scope);

  // TODO: implement evaluate()
  // Use std::uniform_real_distribution from <random>
};
}  // namespace syntax
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__UNIFORM_DISTRIBUTION_HPP_
