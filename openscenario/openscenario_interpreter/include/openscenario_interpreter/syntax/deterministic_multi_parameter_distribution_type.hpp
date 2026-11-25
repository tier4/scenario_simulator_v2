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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__DETERMINISTIC_MULTI_PARAMETER_DISTRIBUTION_TYPE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__DETERMINISTIC_MULTI_PARAMETER_DISTRIBUTION_TYPE_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/value_set_distribution.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   DeterministicMultiParameterDistributionType (OpenSCENARIO XML 1.3.1)

   A deterministic distribution type which can be applied to multiple parameters.

   <xsd:group name="DeterministicMultiParameterDistributionType">
     <xsd:sequence>
       <xsd:element name="ValueSetDistribution" type="ValueSetDistribution"/>
     </xsd:sequence>
   </xsd:group>
*/
struct DeterministicMultiParameterDistributionType : public ValueSetDistribution
{
  explicit DeterministicMultiParameterDistributionType(const pugi::xml_node &, Scope & scope);
};

}  // namespace syntax
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__DETERMINISTIC_MULTI_PARAMETER_DISTRIBUTION_TYPE_HPP_
