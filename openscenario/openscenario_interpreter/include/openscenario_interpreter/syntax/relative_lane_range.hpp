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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_LANE_RANGE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_LANE_RANGE_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/integer.hpp>
#include <optional>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   RelativeLaneRange (OpenSCENARIO XML 1.3)

   <xsd:complexType name="RelativeLaneRange">
     <xsd:attribute name="from" type="Int"/>
     <xsd:attribute name="to" type="Int"/>
   </xsd:complexType>
*/
struct RelativeLaneRange
{
  /*
     The lower limit of the range. Range: [-inf, inf[. Default if omitted: -inf
   */
  const std::optional<Integer> from;

  /*
     The upper limit of the range. Range: ]-inf, inf]. Default if omitted: +inf
   */
  const std::optional<Integer> to;

  explicit RelativeLaneRange(const pugi::xml_node &, Scope &);

  auto evaluate(Integer::value_type value) const -> bool;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_LANE_RANGE_HPP_
