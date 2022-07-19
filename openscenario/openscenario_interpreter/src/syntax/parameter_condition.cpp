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
#include <openscenario_interpreter/syntax/rule.hpp>
#include <openscenario_interpreter/syntax/parameter_condition.hpp>
#include <sstream>
#include <stdexcept>

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
