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

#ifndef SCENARIO_SIMULATOR_EXCEPTION__EXCEPTION_HPP_
#define SCENARIO_SIMULATOR_EXCEPTION__EXCEPTION_HPP_

#include <scenario_simulator_exception/concatenate.hpp>

#include <memory>
#include <stdexcept>
#include <utility>

namespace common
{
inline namespace scenario_simulator_exception
{
struct Error : public std::runtime_error
{
  template <typename... Ts>
  explicit Error(Ts &&... xs) : std::runtime_error(concatenate(std::forward<decltype(xs)>(xs)...))
  {
  }
};

#define DEFINE_ERROR_CATEGORY(TYPENAME) \
  struct TYPENAME : public Error        \
  {                                     \
    using Error::Error;                 \
  }

struct Core
{
  Core(std::string name, std::string description)
  : _name{name}, _description{description}, _anonymous{name.empty()}, _single{true}
  {
  }

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
      int index{0};
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
    std::string prolog =
      "Simulation failure: CustomCommandAction typed \"exitFailure\" was triggered by the ";
    if (_anonymous && _single) {
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
  bool _anonymous{false}, _single{true};
  std::string _name;
  std::string _description;
  std::string _log;
};

struct ScenarioError : public std::runtime_error
{
  ScenarioError(ScenarioError const &) = default;

  explicit ScenarioError(std::string source_name, int index, std::string name)
  : _source_name{source_name},
    _index{index},
    _inner{nullptr},
    _core{nullptr},
    std::runtime_error{ScenarioError::formatName("", index, name)}
  {
  }

  explicit ScenarioError(std::string name, ScenarioError const & inner)
  : _inner{std::make_shared<ScenarioError>(inner)},
    _core{inner._core},
    std::runtime_error{ScenarioError::formatName(inner._source_name, name)}
  {
  }

  explicit ScenarioError(
    std::string source_name, int index, std::string name, ScenarioError const & inner)
  : _source_name{source_name},
    _index{index},
    _inner{std::make_shared<ScenarioError>(inner)},
    _core{inner._core},
    std::runtime_error{ScenarioError::formatName(inner._source_name, index, name)}
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

  auto setCoreSource(std::string name, std::string description) -> void
  {
    if (!_core) _core = std::make_shared<Core>(name, description);
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
  std::shared_ptr<ScenarioError> const _inner;
  std::string _description;
};

// Autoware encountered some problem that led to a simulation failure.
DEFINE_ERROR_CATEGORY(AutowareError);

// Although there were no syntactic errors in the description of the scenario,
// differences in meaning and inconsistencies were found.
DEFINE_ERROR_CATEGORY(SemanticError);

// A problem occurred that interfered with the continuation of the simulation.
DEFINE_ERROR_CATEGORY(SimulationError);

// Metric module detects specification violation in simulation.
DEFINE_ERROR_CATEGORY(SpecificationViolation);

// There is a syntactic error in the description of the scenario. Or you are
// using a feature that is not yet supported by our implementation.
DEFINE_ERROR_CATEGORY(SyntaxError);

#undef DEFINE_ERROR_CATEGORY

#define THROW_ERROR(TYPENAME, ...) throw TYPENAME(__FILE__, ":", __LINE__, ": ", __VA_ARGS__)

#define THROW_SEMANTIC_ERROR(...) /*    */ THROW_ERROR(common::SemanticError, __VA_ARGS__)
#define THROW_SIMULATION_ERROR(...) /*  */ THROW_ERROR(common::SimulationError, __VA_ARGS__)
#define THROW_SPECIFICATION_VIOLATION(...) THROW_ERROR(common::SpecificationViolation, __VA_ARGS__)
#define THROW_SYNTAX_ERROR(...) /*      */ THROW_ERROR(common::SyntaxError, __VA_ARGS__)

#define SPECIFICATION_VIOLATION(...) \
  common::SpecificationViolation(__FILE__, ":", __LINE__, ": ", __VA_ARGS__)
}  // namespace scenario_simulator_exception
}  // namespace common

#endif  // SCENARIO_SIMULATOR_EXCEPTION__EXCEPTION_HPP_
