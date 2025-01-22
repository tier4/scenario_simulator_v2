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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__DETERMINISTIC_PARAMETER_DISTRIBUTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__DETERMINISTIC_PARAMETER_DISTRIBUTION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/deterministic_multi_parameter_distribution.hpp>
#include <openscenario_interpreter/syntax/deterministic_single_parameter_distribution.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   DeterministicParameterDistribution (OpenSCENARIO XML 1.3.1)

   Either a DeterministicMultiParameterDistribution or a DeterministicSingleParameterDistribution

   <xsd:group name="DeterministicParameterDistribution">
     <xsd:choice>
       <xsd:element name="DeterministicMultiParameterDistribution" type="DeterministicMultiParameterDistribution"/>
       <xsd:element name="DeterministicSingleParameterDistribution" type="DeterministicSingleParameterDistribution"/>
     </xsd:choice>
   </xsd:group>
*/
struct DeterministicParameterDistribution : public Group
{
  explicit DeterministicParameterDistribution(const pugi::xml_node &, Scope & scope);
};

DEFINE_LAZY_VISITOR(
  DeterministicParameterDistribution,
  CASE(DeterministicMultiParameterDistribution),   //
  CASE(DeterministicSingleParameterDistribution),  //
);
}  // namespace syntax
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__DETERMINISTIC_PARAMETER_DISTRIBUTION_HPP_
