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

#include <openscenario_interpreter/syntax/init_actions.hpp>
#include <openscenario_interpreter/utility/demangle.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
nlohmann::json & operator<<(nlohmann::json & json, const InitActions & init_actions)
{
  json["GlobalAction"] = nlohmann::json::array();

  for (const auto & init_action : init_actions) {
    if (init_action.is<GlobalAction>()) {
      nlohmann::json global_action;
      global_action = makeTypename(init_action.as<GlobalAction>().type());
      json["GlobalAction"].push_back(global_action);
    }
  }

  json["UserDefinedAction"] = nlohmann::json::array();

  for (const auto & init_action : init_actions) {
    if (init_action.is<UserDefinedAction>()) {
      nlohmann::json user_defined_action;
      user_defined_action = makeTypename(init_action.as<UserDefinedAction>().type());
      json["UserDefinedAction"].push_back(user_defined_action);
    }
  }

  json["Private"] = nlohmann::json::array();

  // for (const auto & init_action : init_actions) {
  //   if (init_action.is<Private>()) {
  //     nlohmann::json global_action;
  //     global_action = makeTypename(init_action.as<Private>().type());
  //     json["Private"].push_back(global_action);
  //   }
  // }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
