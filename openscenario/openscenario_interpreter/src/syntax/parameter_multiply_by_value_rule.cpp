// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/syntax/parameter_multiply_by_value_rule.hpp>
#include <typeindex>
#include <unordered_map>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
ParameterMultiplyByValueRule::ParameterMultiplyByValueRule(
  const pugi::xml_node & node, Scope & scope)
: value(readAttribute<Double>("value", node, scope))
{
}

auto ParameterMultiplyByValueRule::operator()(const Element & target) const -> Element
{
  static const std::unordered_map<
    std::type_index, std::function<Element(const Element &, const Double &)> >
    overloads{
      {typeid(Integer),
       [](auto && target, auto && value) {
         target.template as<Integer>() *= value;
         return target;
       }},

      {typeid(Double),
       [](auto && target, auto && value) {
         target.template as<Double>() *= value;
         return target;
       }},

      {typeid(UnsignedInteger),
       [](auto && target, auto && value) {
         target.template as<UnsignedInteger>() *= value;
         return target;
       }},

      {typeid(UnsignedShort),
       [](auto && target, auto && value) {
         target.template as<UnsignedShort>() *= value;
         return target;
       }},
    };

  const auto iter = overloads.find(target.type());

  if (iter != std::end(overloads)) {
    return std::get<1>(*iter)(target, value);
  } else {
    throw SyntaxError(
      "The parameter specified by attribute 'parameterRef' of type 'ParameterAction' must be "
      "numeric type (double, integer, unsignedInteger or unsignedShort), but ",
      target, " (type ", target.type().name(), ") specified");
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
