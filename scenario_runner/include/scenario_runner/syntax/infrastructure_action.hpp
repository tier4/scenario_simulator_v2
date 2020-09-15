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

#ifndef SCENARIO_RUNNER__SYNTAX__INFRASTRUCTURE_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__INFRASTRUCTURE_ACTION_HPP_

#include <scenario_runner/syntax/traffic_signal_action.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== InfrastructureAction =================================================
 *
 * <xsd:complexType name="InfrastructureAction">
 *   <xsd:all>
 *     <xsd:element name="TrafficSignalAction" type="TrafficSignalAction"/>
 *   </xsd:all>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct InfrastructureAction
  : public All
{
  template<typename Node, typename ... Ts>
  explicit InfrastructureAction(const Node & node, Ts && ... xs)
  {
    defineElement<TrafficSignalAction>("TrafficSignalAction");

    validate(node, std::forward<decltype(xs)>(xs)...);
  }

  auto evaluate() const noexcept
  {
    return unspecified;
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__INFRASTRUCTURE_ACTION_HPP_
