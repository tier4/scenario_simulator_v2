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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_ADD_VALUE_RULE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_ADD_VALUE_RULE_HPP_

#include <openscenario_interpreter/reader/attribute.hpp>

#include <typeindex>
#include <unordered_map>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ParameterAddValueRule --------------------------------------------------
 *
 * <xsd:complexType name="ParameterAddValueRule">
 *   <xsd:attribute name="value" type="Double" use="required"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ParameterAddValueRule
{
  const Double value;

  template<typename ... Ts>
  explicit ParameterAddValueRule(Ts && ... xs)
  : value(readAttribute<Double>("value", std::forward<decltype(xs)>(xs)...))
  {}

  auto operator()(const Element & target) const
  {
    static const std::unordered_map<
      std::type_index,
      std::function<Element(const Element &, const Double &)>
    >
    overloads
    {
      {
        typeid(Integer), [](auto && target, auto && value)
        {
          target.template as<Integer>() += value;
          return target;
        }
      },

      {
        typeid(Double), [](auto && target, auto && value)
        {
          target.template as<Double>() += value;
          return target;
        }
      },

      {
        typeid(UnsignedInteger), [](auto && target, auto && value)
        {
          target.template as<UnsignedInteger>() += value;
          return target;
        }
      },

      {
        typeid(UnsignedShort), [](auto && target, auto && value)
        {
          target.template as<UnsignedShort>() += value;
          return target;
        }
      },
    };

    const auto iter {overloads.find(target.type())};

    if (iter != std::end(overloads)) {
      return std::get<1>(* iter)(target, value);
    } else {
      std::stringstream ss {};
      ss << "The parameter specified by attrribute 'parameterRef' of type 'ParameterAction' ";
      ss << "must be numeric type (double, integer, unsignedInteger or unsignedShort), but ";
      ss << target << " (type " << target.type().name() << ") specified";
      throw SyntaxError(ss.str());
    }
  }
};
}  // inline namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_ADD_VALUE_RULE_HPP_
