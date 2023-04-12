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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__DISTRIBUTION_RANGE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__DISTRIBUTION_RANGE_HPP_

#include <openscenario_interpreter/parameter_distribution.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/range.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- DistributionRange 1.2 --------------------------------------------------
 *
 *  <xsd:complexType name="DistributionRange">
 *    <xsd:all>
 *      <xsd:element name="Range" type="Range"/>
 *    </xsd:all>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */

struct DistributionRange : private Scope, public ComplexType, public SingleParameterDistributionBase
{
  const Double step_width;

  const Range range;

  explicit DistributionRange(const pugi::xml_node &, Scope &);

  auto derive() -> SingleUnnamedParameterDistribution override;

  auto derive(size_t index, size_t total_size) -> Object
  {
    return make<Double>(range.lower_limit + step_width * index);
  }

  auto getNumberOfDeriveScenarios() const -> size_t override;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__DISTRIBUTION_RANGE_HPP_
