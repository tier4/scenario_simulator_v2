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

#include <openscenario_interpreter/syntax/scenario_definition.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto ScenarioDefinition::complete() -> bool { return storyboard.complete(); }

auto ScenarioDefinition::evaluate() -> Element
{
  road_network.evaluate();
  storyboard.evaluate();
  updateFrame();
  return storyboard.current_state;
}

std::ostream & operator<<(std::ostream & os, const ScenarioDefinition & datum)
{
  nlohmann::json json;

  return os << (json << datum).dump(2);
}

nlohmann::json & operator<<(nlohmann::json & json, const ScenarioDefinition & datum)
{
  json["Storyboard"] << datum.storyboard;

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
