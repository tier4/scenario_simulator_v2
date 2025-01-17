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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_VALUE_DISTRIBUTION_DEFINITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_VALUE_DISTRIBUTION_DEFINITION_HPP_

#include <openscenario_interpreter/syntax/parameter_value_distribution.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   ParameterValueDistributionDefinition (OpenSCENARIO XML 1.3.1)

   A marker stating that the OpenSCENARIO file is a parameter value distribution.

   <xsd:group name="ParameterValueDistributionDefinition">
     <xsd:sequence>
       <xsd:element name="ParameterValueDistribution" type="ParameterValueDistribution"/>
     </xsd:sequence>
   </xsd:group>
*/
struct ParameterValueDistributionDefinition : public ParameterValueDistribution
{
  explicit ParameterValueDistributionDefinition(const pugi::xml_node &, Scope &);
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_VALUE_DISTRIBUTION_DEFINITION_HPP_
