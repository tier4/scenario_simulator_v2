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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__DETERMINISTIC_SINGLE_PARAMETER_DISTRIBUTION_TYPE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__DETERMINISTIC_SINGLE_PARAMETER_DISTRIBUTION_TYPE_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/distribution_range.hpp>
#include <openscenario_interpreter/syntax/distribution_set.hpp>
#include <openscenario_interpreter/syntax/user_defined_distribution.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   DeterministicSingleParameterDistributionType (OpenSCENARIO XML 1.3.1)

   A deterministic distribution type which can be applied to a single parameter.

   <xsd:group name="DeterministicSingleParameterDistributionType">
     <xsd:choice>
       <xsd:element name="DistributionSet" type="DistributionSet"/>
       <xsd:element name="DistributionRange" type="DistributionRange"/>
       <xsd:element name="UserDefinedDistribution" type="UserDefinedDistribution"/>
     </xsd:choice>
   </xsd:group>
*/
struct DeterministicSingleParameterDistributionType : public Group
{
  explicit DeterministicSingleParameterDistributionType(const pugi::xml_node &, Scope & scope);
};

DEFINE_LAZY_VISITOR(
  DeterministicSingleParameterDistributionType,
  CASE(DistributionSet),          //
  CASE(DistributionRange),        //
  CASE(UserDefinedDistribution),  //
);
}  // namespace syntax
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__DETERMINISTIC_SINGLE_PARAMETER_DISTRIBUTION_TYPE_HPP_
