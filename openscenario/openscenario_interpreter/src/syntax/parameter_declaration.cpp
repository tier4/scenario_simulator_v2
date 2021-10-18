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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/syntax/parameter_declaration.hpp>
#include <string>
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
ParameterDeclaration::ParameterDeclaration(
  const openscenario_interpreter_msgs::msg::ParameterDeclaration & message)
: name(message.name),                      //
  parameter_type(message.parameter_type),  //
  value(message.value)
{
}

ParameterDeclaration::ParameterDeclaration(const pugi::xml_node & node, Scope & scope)
: name(readAttribute<String>("name", node, scope)),
  parameter_type(readAttribute<ParameterType>("parameterType", node, scope)),
  value(readAttribute<String>("value", node, scope))
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
    throw SyntaxError(
      "In parameter names, usage of symbols is restricted. Symbols that must not be used are:\n"
      "  - \" \" (blank space)\n"
      "  - $\n"
      "  - \'\n"
      "  - \"\n");
  } else {
    scope.insert(name, evaluate());
  }
}

auto ParameterDeclaration::evaluate() const -> Element
{
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
}
}  // namespace syntax
}  // namespace openscenario_interpreter
