// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_SET_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_SET_ACTION_HPP_

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <typeindex>
#include <unordered_map>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- SetAction --------------------------------------------------------------
 *
 *  <xsd:complexType name="ParameterSetAction">
 *    <xsd:attribute name="value" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ParameterSetAction : private Scope
{
  const String parameter_ref;

  const String value;

  template <typename Node>
  explicit ParameterSetAction(const Node & node, Scope & outer_scope, const String & parameter_ref)
  : Scope(outer_scope),
    parameter_ref(parameter_ref),
    value(readAttribute<String>("value", node, localScope()))
  {
  }

  static constexpr auto accomplished() noexcept { return true; }

  auto run() const -> void
  {
    // clang-format off
    static const std::unordered_map<
      std::type_index, std::function<void(const Element &, const String &)>> overloads
    {
      { typeid(Boolean),         [](const Element & parameter, auto && value) { parameter.as<Boolean        >() = boost::lexical_cast<Boolean        >(value); } },
      { typeid(Double),          [](const Element & parameter, auto && value) { parameter.as<Double         >() = boost::lexical_cast<Double         >(value); } },
      { typeid(Integer),         [](const Element & parameter, auto && value) { parameter.as<Integer        >() = boost::lexical_cast<Integer        >(value); } },
      { typeid(String),          [](const Element & parameter, auto && value) { parameter.as<String         >() =                                      value ; } },
      { typeid(UnsignedInteger), [](const Element & parameter, auto && value) { parameter.as<UnsignedInteger>() = boost::lexical_cast<UnsignedInteger>(value); } },
      { typeid(UnsignedShort),   [](const Element & parameter, auto && value) { parameter.as<UnsignedShort  >() = boost::lexical_cast<UnsignedShort  >(value); } },
    };
    // clang-format on

    const auto parameter = findElement(parameter_ref);

    overloads.at(parameter.type())(parameter, value);
  }

  static auto start() noexcept -> void {}
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_SET_ACTION_HPP_
