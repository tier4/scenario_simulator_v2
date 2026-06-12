// Copyright 2015 TIER IV, Inc. All rights reserved.
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
  try {
    local().ref(parameter_ref);
  } catch (const SyntaxError &) {
    if (const String raw = node.attribute("parameterRef").value();
        raw != parameter_ref and not raw.empty() and raw.front() == '$') {
      throw SyntaxError(
        "No parameter named ", std::quoted(parameter_ref), " is declared. Note that the value ",
        std::quoted(raw), " given for attribute parameterRef of ParameterCondition is a parameter ",
        "reference, so it is substituted with the value of parameter ", std::quoted(raw.substr(1)),
        " before lookup. The attribute parameterRef must be the name of a parameter. Did you ",
        "mean parameterRef=\"", raw.substr(1), "\"?");
    } else {
      throw SyntaxError(
        "No parameter named ", std::quoted(parameter_ref),
        " is declared (given for attribute parameterRef of ParameterCondition)");
    }
  }
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

  description << "The value of parameter " << std::quoted(parameter_ref) << " = ";

  // The description is diagnostic information, so it must not throw even if
  // the parameter reference is unresolvable (cf. publishCurrentContext).
  try {
    description << local().ref(parameter_ref);
  } catch (const SyntaxError &) {
    description << "<no such parameter>";
  }

  description << " " << rule << " " << value << "?";

  return description.str();
}

auto ParameterCondition::evaluate() const -> Object
{
  // Note: an unresolvable parameter_ref makes `Scope::ref` throw
  // NoSuchVariableNamed (a SyntaxError), which is already a reasonable error
  // for the scenario to fail with, so it is not caught here.
  if (const auto parameter = local().ref(parameter_ref); not parameter) {
    THROW_SYNTAX_ERROR(parameter_ref, " cannot be found from this scope");
  } else {
    return asBoolean(compare(parameter, rule, value));
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
