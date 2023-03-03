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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/parameter_declaration.hpp>
#include <string>
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto checkName(const std::string & name) -> decltype(auto)
{
  auto includes = [](const std::string & name, const std::vector<char> & chars) {
    return std::any_of(std::begin(chars), std::end(chars), [&](const auto & each) {
      return name.find(each) != std::string::npos;
    });
  };

  if (name.substr(0, 3) == "OSC") {
    throw SyntaxError(
      "Parameter names starting with \"OSC\" are reserved for special use in future versions "
      "of OpenSCENARIO. Generally, it is forbidden to use the OSC prefix.");
  } else if (includes(name, {' ', '$', '\'', '"'})) {
    // throw SyntaxError(
    //   "In parameter names, usage of symbols is restricted. Symbols that must not be used are: "
    //   "whitespace, dollar-sign, single-quote, double-quote. Given parameter name is ",
    //   std::quoted(name));
    return name;
  } else {
    return name;
  }
}

ParameterDeclaration::ParameterDeclaration(const pugi::xml_node & node, Scope & scope)
: name(readAttribute<String>("name", node, scope)),
  parameter_type(readAttribute<ParameterType>("parameterType", node, scope)),
  value(readAttribute<String>("value", node, scope))
{
  traverse<0, unbounded>(
    node, "ConstraintGroup", [&](auto && node) { constraint_groups.emplace_back(node, scope); });
  scope.insert(checkName(name), evaluate());
}

auto ParameterDeclaration::evaluate() const -> Object
{
  auto get_value = [this]() -> Object {
    // clang-format off
    switch (parameter_type) {
      case ParameterType::BOOLEAN:        return make<Boolean      >(value);
      case ParameterType::DATE_TIME:      return make<String       >(value);
      case ParameterType::DOUBLE:         return make<Double       >(value);
      case ParameterType::INTEGER:        return make<Integer      >(value);
      case ParameterType::STRING:         return make<String       >(value);
      case ParameterType::UNSIGNED_INT:   return make<UnsignedInt  >(value);
      case ParameterType::UNSIGNED_SHORT: return make<UnsignedShort>(value);
      default:
        return unspecified;
    }
    // clang-format on
  };

  if (
    constraint_groups.empty() or
    std::any_of(
      std::begin(constraint_groups), std::end(constraint_groups),
      [&](auto && constraint_group) { return constraint_group.evaluate(get_value()); })) {
    return get_value();
  } else {
    std::stringstream ss;
    ss << "The parameter " << std::quoted(name) << " was assigned to \"" << get_value()
       << "\". Please specify a value within the constraints, because the value does not fit the "
          "constraints.";
    throw common::SyntaxError(ss.str());
  }
}

}  // namespace syntax
}  // namespace openscenario_interpreter
