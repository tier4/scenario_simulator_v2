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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__HISTOGRAM_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__HISTOGRAM_HPP_

#include <openscenario_interpreter/parameter_distribution.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/histogram_bin.hpp>
#include <random>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   Histogram (OpenSCENARIO XML 1.3.1)

   Histogram which can be applied to a single parameter.

   <xsd:complexType name="Histogram">
     <xsd:sequence>
       <xsd:element name="Bin" type="HistogramBin" maxOccurs="unbounded"/>
     </xsd:sequence>
   </xsd:complexType>
*/
struct Histogram : public ComplexType, private Scope, public StochasticParameterDistributionBase
{
  /**
   * Note: HistogramBin must be stored in continuous range and ascending order to `bins`
   *       due to restriction of `BinAdapter`
   */
  const std::list<HistogramBin> bins;

  std::piecewise_constant_distribution<Double::value_type> distribute;

  explicit Histogram(const pugi::xml_node &, Scope & scope);

  auto derive() -> Object override;
};
}  // namespace syntax
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__HISTOGRAM_HPP_
