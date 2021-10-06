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

#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/user_defined_value_condition.hpp>
#include <regex>

namespace openscenario_interpreter
{
inline namespace syntax
{
UserDefinedValueCondition::UserDefinedValueCondition(const pugi::xml_node & node, Scope & scope)
: name(readAttribute<String>("name", node, scope)),
  value(readAttribute<String>("value", node, scope)),
  compare(readAttribute<Rule>("rule", node, scope))
{
  static const std::regex pattern{R"(([^\.]+)\.(.+))"};

  std::smatch result;

  if (std::regex_match(name, result, pattern)) {
    const std::unordered_map<std::string, std::function<std::string()>> dispatch{
      std::make_pair("currentState", [result]() { return evaluateCurrentState(result.str(1)); }),
    };
    evaluateValue = dispatch.at(result.str(2));  // XXX catch
  } else {
    throw SyntaxError(__FILE__, ":", __LINE__);
  }
}

auto UserDefinedValueCondition::description() const -> String
{
  std::stringstream description;

  description << "Is the " << name << " (= " << result << ") is " << compare << " " << value << "?";

  return description.str();
}

auto UserDefinedValueCondition::evaluate() -> Element
{
  return asBoolean(compare(result = evaluateValue(), value));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
