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

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/open_scenario_category.hpp>
#include <openscenario_interpreter/syntax/openscenario.hpp>
#include <openscenario_interpreter/syntax/scenario_definition.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
OpenScenario::OpenScenario(const boost::filesystem::path & pathname)
: Scope(pathname),
  file_header(
    readElement<FileHeader>("FileHeader", load(global().pathname).child("OpenSCENARIO"), local())),
  category(readElement<OpenScenarioCategory>("OpenSCENARIO", script, local())),
  frame(0)
{
}

auto OpenScenario::complete() const -> bool { return category.as<ScenarioDefinition>().complete(); }

auto OpenScenario::evaluate() -> Object
{
  ++frame;
  return category.evaluate();
}

auto OpenScenario::load(const boost::filesystem::path & filepath) -> const pugi::xml_node &
{
  const auto result = script.load_file(filepath.string().c_str());

  if (not result) {
    throw SyntaxError(result.description(), ": ", filepath);
  } else {
    return script;
  }
}

auto operator<<(nlohmann::json & json, const OpenScenario & datum) -> nlohmann::json &
{
  json["version"] = "1.0";

  json["frame"] = datum.frame;

  // clang-format off
  json["CurrentStates"]["completeState"]   = openscenario_interpreter::complete_state  .use_count() - 1;
  json["CurrentStates"]["runningState"]    = openscenario_interpreter::running_state   .use_count() - 1;
  json["CurrentStates"]["standbyState"]    = openscenario_interpreter::standby_state   .use_count() - 1;
  json["CurrentStates"]["startTransition"] = openscenario_interpreter::start_transition.use_count() - 1;
  json["CurrentStates"]["stopTransition"]  = openscenario_interpreter::stop_transition .use_count() - 1;
  // clang-format on

  if (datum.category.is<ScenarioDefinition>()) {
    json["OpenSCENARIO"] << datum.category.as<ScenarioDefinition>();
  }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
