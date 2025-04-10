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

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/parameter_value_set.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
ParameterValueSet::ParameterValueSet(
  const pugi::xml_node & node, openscenario_interpreter::Scope & scope)
: Scope(scope),
  parameter_assignments(readElements<ParameterAssignment, 1>("ParameterAssignment", node, local()))
{
}

auto ParameterValueSet::evaluate() const -> std::shared_ptr<std::unordered_map<std::string, Object>>
{
  std::shared_ptr<std::unordered_map<std::string, Object>> parameters =
    std::make_shared<std::unordered_map<std::string, Object>>();
  for (const auto & parameter : parameter_assignments) {
    (*parameters)[parameter.parameterRef] = make<String>(parameter.value);
  }
  return parameters;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
