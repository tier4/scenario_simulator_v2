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

#include <openscenario_interpreter/syntax/trigger.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
nlohmann::json & operator<<(nlohmann::json & json, const Trigger & datum)
{
  json["currentValue"] = boost::lexical_cast<std::string>(Boolean(datum.current_value));

  json["ConditionGroup"] = nlohmann::json::array();

  for (const auto & each : datum) {
    nlohmann::json condition_group;
    condition_group << each;
    json["ConditionGroup"].push_back(condition_group);
  }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
