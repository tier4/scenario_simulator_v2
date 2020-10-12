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

#ifndef SCENARIO_RUNNER__SYNTAX__SET_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__SET_ACTION_HPP_

#include <scenario_runner/reader/attribute.hpp>

#include <utility>

namespace scenario_runner
{
inline namespace syntax
{
/* ---- SetAction --------------------------------------------------------------
 *
 * <xsd:complexType name="ParameterSetAction">
 *   <xsd:attribute name="value" type="String" use="required"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct SetAction
{
  Scope inner_scope;

  const String parameter_ref;

  const String value;

  template<typename Node, typename Scope>
  explicit SetAction(const Node & node, Scope & outer_scope, const String & parameter_ref)
  : inner_scope(outer_scope),
    parameter_ref(parameter_ref),
    value(readAttribute<String>("value", node, inner_scope))
  {}

  auto start()
  {
    return unspecified;
  }

  static constexpr std::true_type accomplished {};
};
}  // inline namespace syntax
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__SET_ACTION_HPP_
