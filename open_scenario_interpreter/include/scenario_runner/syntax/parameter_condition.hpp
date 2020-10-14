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

#ifndef SCENARIO_RUNNER__SYNTAX__PARAMETER_CONDITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__PARAMETER_CONDITION_HPP_

#include <scenario_runner/syntax/rule.hpp>

#include <typeindex>
#include <unordered_map>
#include <utility>

namespace scenario_runner
{
inline namespace syntax
{
/* ---- ParameterCondition -----------------------------------------------------
 *
 * <xsd:complexType name="ParameterCondition">
 *   <xsd:attribute name="parameterRef" type="String" use="required"/>
 *   <xsd:attribute name="value" type="String" use="required"/>
 *   <xsd:attribute name="rule" type="Rule" use="required"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ParameterCondition
{
  const String parameter_ref;

  const String value;

  const Rule compare;

  Scope inner_scope;

  template<typename Node>
  explicit ParameterCondition(const Node & node, Scope & outer_scope)
  : parameter_ref(readAttribute<String>("parameterRef", node, outer_scope)),
    value(readAttribute<String>("value", node, outer_scope)),
    compare(readAttribute<Rule>("rule", node, outer_scope)),
    inner_scope(outer_scope)
  {}

  auto evaluate() const
  {
    const auto target {
      inner_scope.parameters.at(parameter_ref)
    };

    static const std::unordered_map<
      std::type_index,
      std::function<bool(const Rule, const Element &, const String &)>
    >
    overloads
    {
      {
        typeid(Integer), [](auto && compare, auto && lhs, auto && rhs)
        {
          return compare(lhs.template as<Integer>(), boost::lexical_cast<Integer>(rhs));
        }
      },

      {
        typeid(Double), [](auto && compare, auto && lhs, auto && rhs)
        {
          return compare(lhs.template as<Double>(), boost::lexical_cast<Double>(rhs));
        }
      },

      {
        typeid(String), [](auto && compare, auto && lhs, auto && rhs)
        {
          return compare(lhs.template as<String>(), rhs);
        }
      },

      {
        typeid(UnsignedInteger), [](auto && compare, auto && lhs, auto && rhs)
        {
          return compare(
            lhs.template as<UnsignedInteger>(), boost::lexical_cast<UnsignedInteger>(rhs));
        }
      },

      {
        typeid(UnsignedShort), [](auto && compare, auto && lhs, auto && rhs)
        {
          return compare(lhs.template as<UnsignedShort>(), boost::lexical_cast<UnsignedShort>(rhs));
        }
      },

      {
        typeid(Boolean), [](auto && compare, auto && lhs, auto && rhs)
        {
          return compare(lhs.template as<Boolean>(), boost::lexical_cast<Boolean>(rhs));
        }
      },
    };

    const auto iter {
      overloads.find(target.type())
    };

    if (iter != std::end(overloads)) {
      std::cout << "ParameterCondition: " << target << " " << compare << " " << value << " => ";
      const auto result = std::get<1>(* iter)(compare, target, value) ? true_v : false_v;
      std::cout << result << std::endl;
      return result;
    } else {
      THROW_IMPLEMENTATION_FAULT();
    }
  }
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__PARAMETER_CONDITION_HPP_
