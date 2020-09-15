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

#ifndef SCENARIO_RUNNER__SYNTAX__TRAFFIC_SIGNAL_STATE_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__TRAFFIC_SIGNAL_STATE_ACTION_HPP_

#include <scenario_runner/validator/attribute.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== TrafficSignalStateAction =============================================
 *
 * <xsd:complexType name="TrafficSignalStateAction">
 *   <xsd:attribute name="name" type="String" use="required"/>
 *   <xsd:attribute name="state" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct TrafficSignalStateAction
{
  const std::string name;
  const std::string state;

  template<typename Node, typename ... Ts>
  explicit TrafficSignalStateAction(const Node & node, Ts && ...)
  : name{readRequiredAttribute<std::string>(node, "name")},
    state{readRequiredAttribute<std::string>(node, "state")}
  {}

  auto evaluate() const noexcept
  {
    return unspecified;
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__TRAFFIC_SIGNAL_STATE_ACTION_HPP_
