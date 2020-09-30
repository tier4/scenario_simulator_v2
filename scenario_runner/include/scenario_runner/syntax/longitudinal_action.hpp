// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef SCENARIO_RUNNER__SYNTAX__LONGITUDINAL_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__LONGITUDINAL_ACTION_HPP_

#include <scenario_runner/syntax/speed_action.hpp>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== LongitudinalAction ===================================================
 *
 * <xsd:complexType name="LongitudinalAction">
 *   <xsd:choice>
 *     <xsd:element name="SpeedAction" type="SpeedAction"/>
 *     <xsd:element name="LongitudinalDistanceAction" type="LongitudinalDistanceAction"/>
 *   </xsd:choice>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct LongitudinalAction
  : public Element
{
  template<typename Node, typename Scope>
  explicit LongitudinalAction(const Node & node, Scope & scope)
  {
    callWithElements(
      node, "SpeedAction", 0, 1, [&](auto && node)
      {
        return rebind<SpeedAction>(node, scope);
      });

    callWithElements(node, "LongitudinalDistanceAction", 0, 1, THROW_UNSUPPORTED_ERROR(node));
  }

  using Element::evaluate;
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__LONGITUDINAL_ACTION_HPP_
