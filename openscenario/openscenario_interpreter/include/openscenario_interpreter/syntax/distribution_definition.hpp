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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__DISTRIBUTION_DEFINITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__DISTRIBUTION_DEFINITION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/deterministic.hpp>
#include <openscenario_interpreter/syntax/stochastic.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   DistributionDefinition (OpenSCENARIO XML 1.3.1)

   Indicates whether the content defines a deterministic or stochastic parameter distribution.

   <xsd:group name="DistributionDefinition">
     <xsd:choice>
       <xsd:element name="Deterministic" type="Deterministic"/>
       <xsd:element name="Stochastic" type="Stochastic"/>
     </xsd:choice>
   </xsd:group>
*/
struct DistributionDefinition : public Group
{
  explicit DistributionDefinition(const pugi::xml_node &, Scope & scope);
};

DEFINE_LAZY_VISITOR(
  DistributionDefinition,  //
  CASE(Deterministic),     //
  CASE(Stochastic),        //
);

}  // namespace syntax
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__DISTRIBUTION_DEFINITION_HPP_
