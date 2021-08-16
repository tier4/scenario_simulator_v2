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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_CONDITION_HPP_

#include <iomanip>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/rule.hpp>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ParameterCondition -----------------------------------------------------
 *
 *  <xsd:complexType name="ParameterCondition">
 *    <xsd:attribute name="parameterRef" type="String" use="required"/>
 *    <xsd:attribute name="value" type="String" use="required"/>
 *    <xsd:attribute name="rule" type="Rule" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ParameterCondition : private Scope
{
  const String parameter_ref;

  const String value;

  const Rule compare;

  template <typename Node>
  explicit ParameterCondition(const Node & node, Scope & current_scope)
  // clang-format off
  : Scope(current_scope),
    parameter_ref(readAttribute<String>("parameterRef", node, localScope())),
    value        (readAttribute<String>("value",        node, localScope())),
    compare      (readAttribute<Rule>  ("rule",         node, localScope()))
  // clang-format on
  {
  }

  auto description() const
  {
    std::stringstream description;

    description << "The value of parameter " << std::quoted(parameter_ref) << " = "
                << localScope().findElement(parameter_ref) << " " << compare << " " << value << "?";

    return description.str();
  }

  auto evaluate() const
  {
    static const std::unordered_map<
      std::type_index,  //
      std::function<bool(const Rule, const Element &, const String &)>>
      overloads{
        // clang-format off
        { typeid(Boolean        ), [](auto && compare, auto && lhs, auto && rhs) { return compare(lhs.template as<Boolean        >(), boost::lexical_cast<Boolean        >(rhs)); } },
        { typeid(Double         ), [](auto && compare, auto && lhs, auto && rhs) { return compare(lhs.template as<Double         >(), boost::lexical_cast<Double         >(rhs)); } },
        { typeid(Integer        ), [](auto && compare, auto && lhs, auto && rhs) { return compare(lhs.template as<Integer        >(), boost::lexical_cast<Integer        >(rhs)); } },
        { typeid(String         ), [](auto && compare, auto && lhs, auto && rhs) { return compare(lhs.template as<String         >(),                                      rhs ); } },
        { typeid(UnsignedInteger), [](auto && compare, auto && lhs, auto && rhs) { return compare(lhs.template as<UnsignedInteger>(), boost::lexical_cast<UnsignedInteger>(rhs)); } },
        { typeid(UnsignedShort  ), [](auto && compare, auto && lhs, auto && rhs) { return compare(lhs.template as<UnsignedShort  >(), boost::lexical_cast<UnsignedShort  >(rhs)); } },
        // clang-format on
      };

    try {
      const auto & parameter = localScope().findElement(parameter_ref);
      if (!parameter) {
        THROW_SYNTAX_ERROR(parameter_ref, " cannot be found from this scope");
      }
      try {
        return asBoolean(overloads.at(parameter.type())(compare, parameter, value));
      } catch (const std::out_of_range &) {
        throw SemanticError(
          "No viable operation ", std::quoted(boost::lexical_cast<String>(compare)),
          " with parameter ", std::quoted(parameter_ref), " and value ", std::quoted(value));
      }
    } catch (const std::out_of_range &) {
      throw SemanticError("No such parameter ", std::quoted(parameter_ref));
    }
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_CONDITION_HPP_
