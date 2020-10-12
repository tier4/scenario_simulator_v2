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

#ifndef SCENARIO_RUNNER__SYNTAX__PARAMETER_MULTIPLY_BY_VALUE_RULE_HPP_
#define SCENARIO_RUNNER__SYNTAX__PARAMETER_MULTIPLY_BY_VALUE_RULE_HPP_

#include <scenario_runner/reader/attribute.hpp>

#include <utility>

namespace scenario_runner
{
inline namespace syntax
{
/* ---- ParameterMultiplyByValueRule -------------------------------------------
 *
 * <xsd:complexType name="ParameterMultiplyByValueRule">
 *   <xsd:attribute name="value" type="Double" use="required"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ParameterMultiplyByValueRule
{
  const Double value;

  template<typename ... Ts>
  explicit ParameterMultiplyByValueRule(Ts && ... xs)
  : value(readAttribute<Double>("value", std::forward<decltype(xs)>(xs)...))
  {}
};
}  // inline namespace syntax
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__PARAMETER_MULTIPLY_BY_VALUE_RULE_HPP_
