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

#include <openscenario_interpreter/syntax/maneuver_group.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
nlohmann::json & operator<<(nlohmann::json & json, const ManeuverGroup & datum)
{
  json["name"] = datum.name;

  json["state"] = boost::lexical_cast<std::string>(datum.state());

  json["currentExecutionCount"] = datum.current_execution_count;
  json["maximumExecutionCount"] = datum.maximum_execution_count;

  json["Maneuver"] = nlohmann::json::array();

  // for (const auto & each : datum) {
  //   nlohmann::json act;
  //   act << each.as<Maneuver>();
  //   json["Maneuver"].push_back(act);
  // }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
