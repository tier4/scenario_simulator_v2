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

#include <openscenario_interpreter/syntax/action.hpp>
#include <openscenario_interpreter/syntax/custom_command_action.hpp>
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
    std::make_pair("GlobalAction",      [&](auto && node) { return global_actions.push_back(make<GlobalAction>(node, scope)); }),
    std::make_pair("UserDefinedAction", [&](auto && node) { return user_defined_actions.push_back(make<UserDefinedAction>(node, scope)); }),
    std::make_pair("Private",           [&](auto && node) { return privates.push_back(make<Private>(node, scope)); })
    // clang-format on
  };

  for (const auto & each : node.children()) {
    const auto iter = dispatcher.find(each.name());
    if (iter != std::end(dispatcher)) {
      std::get<1> (*iter)(each);
    }
  }
}

auto InitActions::evaluate() -> Object { return StoryboardElement::evaluate(); }

auto InitActions::accomplished() const -> bool
{
  auto global_actions_accomplished = std::all_of(
    global_actions.begin(), global_actions.end(),
    [=](const Object & e) { return e.as<GlobalAction>().accomplished(); });
  auto user_defined_actions_accomplished = std::all_of(
    user_defined_actions.begin(), user_defined_actions.end(),
    [=](const Object & e) { return e.as<UserDefinedAction>().accomplished(); });
  auto privates_accomplished = std::all_of(privates.begin(), privates.end(), [=](const Object & e) {
    return e.as<Private>().accomplished();
  });

  return global_actions_accomplished and user_defined_actions_accomplished and
         privates_accomplished;
}

// this function should be called by StoryboardElement
auto InitActions::run() -> void { runNonInstantaneousActions(); }

auto InitActions::endsImmediately() const -> bool
{
  auto global_ends_immediately = std::all_of(
    global_actions.begin(), global_actions.end(),
    [=](const Object & e) { return e.as<GlobalAction>().endsImmediately(); });
  // In this class, there are some implementations that assume all global actions are instantaneous actions.
  assert(global_ends_immediately);
  auto user_defined_actions_ends_immediately = std::all_of(
    user_defined_actions.begin(), user_defined_actions.end(),
    [=](const Object & e) { return e.as<UserDefinedAction>().endsImmediately(); });
  auto private_actions_ends_immediately = std::all_of(
    privates.begin(), privates.end(),
    [=](const Object & e) { return e.as<Private>().endsImmediately(); });

  return global_ends_immediately or user_defined_actions_ends_immediately or
         private_actions_ends_immediately;
}

// this function should be called by StoryboardElement
auto InitActions::start() -> void { startNonInstantaneousActions(); }

// this function should be called before simulation time starts
auto InitActions::startInstantaneousActions() -> void
{
  for (auto && e : global_actions) {
    e.as<GlobalAction>().start();
  }

  std::size_t index{0};
  for (auto && element : user_defined_actions) {
    try {
      auto & user_defined_action = element.as<UserDefinedAction>();
      if (user_defined_action.endsImmediately()) {
        user_defined_action.start();
      }
      ++index;
    } catch (const SpecialAction<EXIT_FAILURE> & action) {
      throw SpecialAction<EXIT_FAILURE>("Actions", "UserDefinedAction", index, "Action");
    }
  }

  for (auto && e : privates) {
    e.as<Private>().startInstantaneousActions();
  }
}

auto InitActions::startNonInstantaneousActions() -> void
{
  // we don't call global actions here, because they are all instantaneous actions
  for (auto && e : user_defined_actions) {
    auto & user_defined_action = e.as<UserDefinedAction>();
    if (not user_defined_action.endsImmediately()) {
      user_defined_action.start();
    }
  }
  for (auto && e : privates) {
    e.as<Private>().startNonInstantaneousActions();
  }
}

// this function should be called before simulation time starts and after executing startInstantaneousActions()
auto InitActions::runInstantaneousActions() -> void
{
  for (auto && e : global_actions) {
    e.as<GlobalAction>().run();
  }
  for (auto && e : user_defined_actions) {
    auto & user_defined_action = e.as<UserDefinedAction>();
    if (user_defined_action.endsImmediately()) {
      user_defined_action.run();
    }
  }
  for (auto && e : privates) {
    e.as<Private>().runInstantaneousActions();
  }
}

auto InitActions::runNonInstantaneousActions() -> void
{
  // we don't call global actions here, because they are all instantaneous actions
  for (auto && e : user_defined_actions) {
    auto & user_defined_action = e.as<UserDefinedAction>();
    if (not user_defined_action.endsImmediately()) {
      user_defined_action.run();
    }
  }
  for (auto && e : privates) {
    e.as<Private>().runNonInstantaneousActions();
  }
}

auto operator<<(nlohmann::json & json, const InitActions & init_actions) -> nlohmann::json &
{
  json["GlobalAction"] = nlohmann::json::array();

  for (const auto & init_action : init_actions.global_actions) {
    nlohmann::json action;
    action["type"] = makeTypename(init_action.as<GlobalAction>().type());
    json["GlobalAction"].push_back(action);
  }

  json["UserDefinedAction"] = nlohmann::json::array();

  for (const auto & init_action : init_actions.user_defined_actions) {
    nlohmann::json action;
    action["type"] = makeTypename(init_action.as<UserDefinedAction>().type());
    json["UserDefinedAction"].push_back(action);
  }

  json["Private"] = nlohmann::json::array();

  for (const auto & init_action : init_actions.privates) {
    nlohmann::json action;
    action << init_action.as<Private>();
    json["Private"].push_back(action);
  }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
