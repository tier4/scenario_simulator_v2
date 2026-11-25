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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__STOCHASTIC_DISTRIBUTION_TYPE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__STOCHASTIC_DISTRIBUTION_TYPE_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/histogram.hpp>
#include <openscenario_interpreter/syntax/log_normal_distribution.hpp>
#include <openscenario_interpreter/syntax/normal_distribution.hpp>
#include <openscenario_interpreter/syntax/poisson_distribution.hpp>
#include <openscenario_interpreter/syntax/probability_distribution_set.hpp>
#include <openscenario_interpreter/syntax/uniform_distribution.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   StochasticDistributionType (OpenSCENARIO XML 1.3.1)

   Container for a stochastic distribution type which can be applied to a single parameter.

   <xsd:group name="StochasticDistributionType">
     <xsd:choice>
       <xsd:element name="ProbabilityDistributionSet" type="ProbabilityDistributionSet"/>
       <xsd:element name="NormalDistribution" type="NormalDistribution"/>
       <xsd:element name="LogNormalDistribution" type="LogNormalDistribution"/>
       <xsd:element name="UniformDistribution" type="UniformDistribution"/>
       <xsd:element name="PoissonDistribution" type="PoissonDistribution"/>
       <xsd:element name="Histogram" type="Histogram"/>
       <xsd:element name="UserDefinedDistribution" type="UserDefinedDistribution"/>
     </xsd:choice>
   </xsd:group>
*/
struct StochasticDistributionType : public Group
{
  explicit StochasticDistributionType(const pugi::xml_node &, Scope & scope);
};

DEFINE_LAZY_VISITOR(
  StochasticDistributionType,
  CASE(ProbabilityDistributionSet),  //
  CASE(NormalDistribution),          //
  CASE(LogNormalDistribution),       //
  CASE(UniformDistribution),         //
  CASE(PoissonDistribution),         //
  CASE(Histogram)                    //
  //  CASE(UserDefinedDistribution),     //
);
}  // namespace syntax
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__STOCHASTIC_DISTRIBUTION_TYPE_HPP_
