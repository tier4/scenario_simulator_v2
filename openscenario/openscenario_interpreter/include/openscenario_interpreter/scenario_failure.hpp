// Copyright 2023 TIER IV, Inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__SCENARIO_FAILURE_HPP_
#define OPENSCENARIO_INTERPRETER__SCENARIO_FAILURE_HPP_

#include <openscenario_interpreter/utility/demangle.hpp>
#include <scenario_simulator_exception/exception.hpp>

#include <exception>
#include <iomanip>
#include <stdexcept>

namespace openscenario_interpreter
{

struct Core
{
  Core(bool init_action_source) : _init_action{init_action_source} {}

  Core(std::vector<std::pair<std::string, std::string>> conditions_list)
  : _single{conditions_list.size() == 1}
  {
    _anonymous = std::any_of(conditions_list.begin(), conditions_list.end(), [](const auto & pair) {
      return pair.first.empty();
    });

    if (_single) {
      _name = "\"" + conditions_list.at(0).first + "\"";
      _description = conditions_list.at(0).second;
    } else {
      size_t index{0};
      for (const auto & condition : conditions_list) {
        _name += "\"" + condition.first + "\", ";
        if (!_anonymous) {
          _description += "{\"" + condition.first + "\": " + condition.second + "}, ";
        } else {
          _description += "{\"" + std::to_string(index++) + "\": " + condition.second + "}, ";
        }
      }
      _name.erase(_name.length() - 2);
      _description.erase(_description.length() - 2);
    }
  }

  auto log() const -> const std::string & { return _log; }

  auto logUpdate(std::string path) -> std::string
  {
    std::string prolog = "CustomCommandAction typed \"exitFailure\" was triggered by the ";
    if (_init_action) {
      _log = prolog + "anonymous init action (" + path + ")";
    } else if (_anonymous && _single) {
      _log = prolog + "anonymous condition (" + path + ".Condition[0]): " + _description;
    } else if (_single) {
      _log = prolog + "Condition named " + _name + ": " + _description;
    } else if (_anonymous && !_single) {
      _log = prolog + "anonymous conditions (" + path + ".Condition[...]): " + _description;
    } else {
      _log = prolog + "Conditions named {" + _name + "}: " + _description;
    }
    return _log;
  }

private:
  bool _anonymous{false}, _single{true}, _init_action{false};
  std::string _name;
  std::string _description;
  std::string _log;
};

struct ScenarioFailure : public std::runtime_error
{
  ScenarioFailure(ScenarioFailure const &) = default;

  explicit ScenarioFailure(std::string source_name, int index, std::string name)
  : _source_name{source_name},
    _index{index},
    _inner{nullptr},
    _core{nullptr},
    std::runtime_error{ScenarioFailure::formatName("", index, name)}
  {
  }

  explicit ScenarioFailure(std::string name, ScenarioFailure const & inner)
  : _inner{std::make_shared<ScenarioFailure>(inner)},
    _core{inner._core},
    std::runtime_error{ScenarioFailure::formatName(inner._source_name, name)}
  {
  }

  explicit ScenarioFailure(
    std::string source_name, int index, std::string name, ScenarioFailure const & inner)
  : _source_name{source_name},
    _index{index},
    _inner{std::make_shared<ScenarioFailure>(inner)},
    _core{inner._core},
    std::runtime_error{ScenarioFailure::formatName(inner._source_name, index, name)}
  {
  }

  static auto formatName(std::string inner_source_name, std::string name) -> std::string
  {
    if (!inner_source_name.empty())
      return name + "." + inner_source_name;
    else
      return name;
  }

  static auto formatName(std::string inner_source_name, int index, std::string name) -> std::string
  {
    if (!inner_source_name.empty())
      return "." + name + "[\"" + inner_source_name + "\"]";
    else
      return "." + name + "[" + std::to_string(index) + "]";
  }

  auto setCoreSource(std::vector<std::pair<std::string, std::string>> conditions_list) -> void
  {
    if (!_core) _core = std::make_shared<Core>(conditions_list);
  }

  auto setInitActionAsSource() -> void
  {
    if (!_core) _core = std::make_shared<Core>(true);
  }

  auto recursiveWhat() const -> std::string
  {
    if (_inner) {
      return std::runtime_error::what() + _inner->recursiveWhat();
    } else {
      return std::runtime_error::what();
    }
  }

  const char * what() const noexcept override
  {
    if (_core) {
      _core->logUpdate(recursiveWhat());
      return _core->log().c_str();
    } else
      return "No error core defined";
  }

  std::string _source_name{""};
  int _index{-1};

private:
  std::shared_ptr<Core> _core;
  std::shared_ptr<ScenarioFailure> const _inner;
  std::string _description;
};

}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SCENARIO_FAILURE_HPP_
