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

#include <typeindex>
#include <unordered_map>
#include <utility>

namespace openscenario_interpreter
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
struct ParameterSetAction
{
  Scope inner_scope;

  const String parameter_ref;

  const String value;

  const std::true_type accomplished {};

  template<typename Node, typename Scope>
  explicit ParameterSetAction(const Node & node, Scope & outer_scope, const String & parameter_ref)
  : inner_scope(outer_scope),
    parameter_ref(parameter_ref),
    value(readAttribute<String>("value", node, inner_scope))
  {}

  auto evaluate() const noexcept(false)
  {
    static const std::unordered_map<
      std::type_index,
      std::function<Element(const Element &, const String &)>
    >
    overloads
    {
      {
        typeid(Integer), [](auto && target, auto && value)
        {
          target.template as<Integer>() = boost::lexical_cast<Integer>(value);
          return target;
        }
      },

      {
        typeid(Double), [](auto && target, auto && value)
        {
          target.template as<Double>() = boost::lexical_cast<Double>(value);
          return target;
        }
      },

      {
        typeid(String), [](auto && target, auto && value)
        {
          target.template as<String>() = value;
          return target;
        }
      },

      {
        typeid(UnsignedInteger), [](auto && target, auto && value)
        {
          target.template as<UnsignedInteger>() = boost::lexical_cast<UnsignedInteger>(value);
          return target;
        }
      },

      {
        typeid(UnsignedShort), [](auto && target, auto && value)
        {
          target.template as<UnsignedShort>() = boost::lexical_cast<UnsignedShort>(value);
          return target;
        }
      },

      {
        typeid(Boolean), [](auto && target, auto && value)
        {
          target.template as<Boolean>() = boost::lexical_cast<Boolean>(value);
          return target;
        }
      },
    };

    const auto target {
      inner_scope.parameters.at(parameter_ref)
    };

    const auto iter {
      overloads.find(target.type())
    };

    if (iter != std::end(overloads)) {
      return std::get<1>(* iter)(target, value);
    } else {
      THROW_IMPLEMENTATION_FAULT();
    }
  }
};
}  // inline namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_SET_ACTION_HPP_
