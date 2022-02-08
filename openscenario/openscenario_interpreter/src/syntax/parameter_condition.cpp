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

#include <iomanip>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/syntax/parameter_condition.hpp>
#include <sstream>
#include <stdexcept>
#include <typeindex>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace syntax
{
ParameterCondition::ParameterCondition(const pugi::xml_node & node, Scope & scope)
: Scope(scope),
  parameter_ref(readAttribute<String>("parameterRef", node, local())),
  value(readAttribute<String>("value", node, local())),
  rule(readAttribute<Rule>("rule", node, local()))
{
}

auto ParameterCondition::compare(const Object & parameter, const Rule & rule, const String & value)
  -> bool
{
  static const std::unordered_map<
    std::type_index,  //
    std::function<bool(const Object &, const Rule, const String &)>>
    overloads{
      // clang-format off
      { typeid(Boolean        ), [](auto && lhs, auto && compare, auto && rhs) { return compare(lhs.template as<Boolean        >(), Boolean        (rhs)); } },
      { typeid(Double         ), [](auto && lhs, auto && compare, auto && rhs) { return compare(lhs.template as<Double         >(), Double         (rhs)); } },
      { typeid(Integer        ), [](auto && lhs, auto && compare, auto && rhs) { return compare(lhs.template as<Integer        >(), Integer        (rhs)); } },
      { typeid(String         ), [](auto && lhs, auto && compare, auto && rhs) { return compare(lhs.template as<String         >(),                 rhs ); } },
      { typeid(UnsignedInteger), [](auto && lhs, auto && compare, auto && rhs) { return compare(lhs.template as<UnsignedInteger>(), UnsignedInteger(rhs)); } },
      { typeid(UnsignedShort  ), [](auto && lhs, auto && compare, auto && rhs) { return compare(lhs.template as<UnsignedShort  >(), UnsignedShort  (rhs)); } },
      // clang-format on
    };

  try {
    return overloads.at(parameter.type())(parameter, rule, value);
  } catch (const std::out_of_range &) {
    throw SemanticError(
      "No viable operation ", std::quoted(boost::lexical_cast<String>(rule)), " with value ",
      std::quoted(boost::lexical_cast<String>(parameter)), " and value ", std::quoted(value));
  }
}

auto ParameterCondition::description() const -> String
{
  std::stringstream description;

  description << "The value of parameter " << std::quoted(parameter_ref) << " = "
              << local().ref(parameter_ref) << " " << rule << " " << value << "?";

  return description.str();
}

auto ParameterCondition::evaluate() const -> Object
{
  try {
    const auto parameter = local().ref(parameter_ref);
    if (not parameter) {
      THROW_SYNTAX_ERROR(parameter_ref, " cannot be found from this scope");
    } else {
      return asBoolean(compare(parameter, rule, value));
    }
  } catch (const std::out_of_range &) {
    throw SemanticError("No such parameter ", std::quoted(parameter_ref));
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
