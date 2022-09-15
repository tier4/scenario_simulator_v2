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

#ifndef OPENSCENARIO_INTERPRETER__HISTOGRAM_HPP_
#define OPENSCENARIO_INTERPRETER__HISTOGRAM_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/histogram_bin.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Histogram --------------------------------------------------------------
 *
 *  <xsd:complexType name="Histogram">
 *    <xsd:sequence>
 *      <xsd:element name="Bin" type="HistogramBin" maxOccurs="unbounded"/>
 *    </xsd:sequence>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Histogram : public ComplexType
{
  const std::list<HistogramBin> bins;

  explicit Histogram(const pugi::xml_node &, Scope & scope);

  // TODO: implement evaluate()
  // Use std::piecewise_constant_distribution from <random>
};
}  // namespace syntax
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__HISTOGRAM_HPP_
