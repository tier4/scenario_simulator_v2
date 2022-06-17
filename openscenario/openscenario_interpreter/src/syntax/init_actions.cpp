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

#include <openscenario_interpreter/syntax/action.hpp>
#include <openscenario_interpreter/syntax/init_actions.hpp>
#include <openscenario_interpreter/syntax/private.hpp>
#include <openscenario_interpreter/syntax/user_defined_action.hpp>
#include <openscenario_interpreter/utility/demangle.hpp>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace syntax
{
InitActions::InitActions(const pugi::xml_node & node, Scope & scope)
{
  std::unordered_map<std::string, std::function<void(const pugi::xml_node & node)>> dispatcher{
    // clang-format off
    std::make_pair("GlobalAction",      [&](auto && node) { return all_elements.push_back(make<GlobalAction>     (node, scope)); }),
    std::make_pair("UserDefinedAction", [&](auto && node) { return all_elements.push_back(make<UserDefinedAction>(node, scope)); }),
    std::make_pair("Private",           [&](auto && node) { return all_elements.push_back(make<Private>          (node, scope)); })
    // clang-format on
  };

  for (const auto & each : node.children()) {
    const auto iter = dispatcher.find(each.name());
    if (iter != std::end(dispatcher)) {
      std::get<1> (*iter)(each);
    }
  }

  for (auto e : all_elements) {
    if (e.is<GlobalAction>()) {
      apply<void>(
        [&](auto && action) {
          if (action.endsImmediately()) {
            instant_elements.push_back(action);
          } else {
            non_instant_elements.push_back(action);
          }
        },
        e.as<GlobalAction>());
    } else if (e.is<UserDefinedAction>()) {
      auto action = e.as<UserDefinedAction>();
      if (action.endsImmediately()) {
        instant_elements.push_back(action);
      } else {
        non_instant_elements.push_back(action);
      }
    } else if (e.is<Private>()) {
      auto push_back_specific_private =
        [&](Elements & elements, std::function<bool(const PrivateAction &)> remove_func) {
          Private local_private = e.as<Private>();
          local_private.private_actions.erase(
            std::remove_if(
              std::begin(local_private.private_actions), std::end(local_private.private_actions),
              remove_func),
            local_private.private_actions.end());
          if (not local_private.private_actions.empty()) {
            elements.push_back(make(local_private));
          }
        };

      push_back_specific_private(
        instant_elements, [](const PrivateAction & e) { return not e.endsImmediately(); });
      push_back_specific_private(
        non_instant_elements, [](const PrivateAction & e) { return e.endsImmediately(); });
    }
  }
}

auto InitActions::evaluate() const -> Object
{
  evaluateInstantly();

  return unspecified;
}

auto InitActions::evaluateInstantly() const -> Object
{
  for (auto && each : instant_elements) {
    each.evaluate();
  }
  return unspecified;
}

auto InitActions::evaluateNonInstantly() const -> Object
{
  for (auto && each : non_instant_elements) {
    each.evaluate();
  }
  return unspecified;
}

auto InitActions::endsImmediately() const -> bool
{
  return std::all_of(all_elements.begin(), all_elements.end(), [=](const Object & e) {
    if (e.is<GlobalAction>()) {
      return e.as<GlobalAction>().endsImmediately();
    } else if (e.is<UserDefinedAction>()) {
      return e.as<UserDefinedAction>().endsImmediately();
    } else if (e.is<Private>()) {
      return e.as<Private>().endsImmediately();
    } else {
      throw UNSUPPORTED_ELEMENT_SPECIFIED(e.type().name());
    }
  });
}

auto operator<<(nlohmann::json & json, const InitActions & init_actions) -> nlohmann::json &
{
  json["GlobalAction"] = nlohmann::json::array();

  for (const auto & init_action : init_actions.all_elements) {
    if (init_action.is<GlobalAction>()) {
      nlohmann::json action;
      action["type"] = makeTypename(init_action.as<GlobalAction>().type());
      json["GlobalAction"].push_back(action);
    }
  }

  json["UserDefinedAction"] = nlohmann::json::array();

  for (const auto & init_action : init_actions.all_elements) {
    if (init_action.is<UserDefinedAction>()) {
      nlohmann::json action;
      action["type"] = makeTypename(init_action.as<UserDefinedAction>().type());
      json["UserDefinedAction"].push_back(action);
    }
  }

  json["Private"] = nlohmann::json::array();

  for (const auto & init_action : init_actions.all_elements) {
    if (init_action.is<Private>()) {
      nlohmann::json action;
      action << init_action.as<Private>();
      json["Private"].push_back(action);
    }
  }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
