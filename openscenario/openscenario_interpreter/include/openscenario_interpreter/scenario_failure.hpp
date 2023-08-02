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

struct ScenarioFailure : public std::runtime_error
{
  struct CoreSource
  {
    CoreSource(bool init_action_source) : init_action_{init_action_source} {}

    CoreSource(std::vector<std::pair<std::string, std::string>> conditions_list)
    : single_{conditions_list.size() == 1}
    {
      anonymous_ = std::any_of(
        conditions_list.begin(), conditions_list.end(),
        [](const auto & pair) { return pair.first.empty(); });

      std::stringstream name, description;
      if (single_) {
        name << std::quoted(conditions_list.front().first);
        description << conditions_list.front().second;
      } else {
        for (size_t index = 0; const auto & condition : conditions_list) {
          name << std::quoted(condition.first) << ", ";
          description << "{" << std::quoted(anonymous_ ? std::to_string(index++) : condition.first)
                      << ": " << condition.second << "}, ";
        }
        name.seekp(-2, std::ios_base::end);
        description.seekp(-2, std::ios_base::end);
      }
      name_ = name.str();
      description_ = description.str();
    }

    auto log() const -> const std::string & { return log_; }

    auto logUpdate(std::string path) -> std::string
    {
      std::stringstream log;
      log << "CustomCommandAction typed " << std::quoted("exitFailure") << " was triggered by the ";
      if (init_action_) {
        log << "anonymous init action (" << path << ")";
      } else if (anonymous_ && single_) {
        log << "anonymous condition (" << path << ".Condition[0]): " << description_;
      } else if (single_) {
        log << "Condition named " << name_ << ": " << description_;
      } else if (anonymous_) {
        log << "anonymous conditions (" << path << ".Condition[...]): " << description_;
      } else {
        log << "Conditions named {" << name_ << "}: " << description_;
      }
      log_ = log.str();
      return log_;
    }

  private:
    bool anonymous_{false}, single_{true}, init_action_{false};
    std::string name_;
    std::string description_;
    std::string log_;
  };

  ScenarioFailure(ScenarioFailure const &) = default;

  // Constructor with no inner object -- first object
  explicit ScenarioFailure(std::string source_name, int index, std::string name)
  : source_name_{source_name},
    index_{index},
    inner_{nullptr},
    core_{nullptr},
    std::runtime_error{ScenarioFailure::formatName("", index, name)}
  {
  }

  // Constructor with inner object and current source (to be forwarded) -- intermediate object
  explicit ScenarioFailure(
    std::string source_name, int index, std::string name, ScenarioFailure const & inner)
  : source_name_{source_name},
    index_{index},
    inner_{std::make_shared<ScenarioFailure>(inner)},
    core_{inner.core_},
    std::runtime_error{ScenarioFailure::formatName(inner.source_name_, index, name)}
  {
  }

  // Constructor with inner but without current source -- last object
  explicit ScenarioFailure(std::string name, ScenarioFailure const & inner)
  : source_name_{""},
    index_{0},
    inner_{std::make_shared<ScenarioFailure>(inner)},
    core_{inner.core_},
    std::runtime_error{ScenarioFailure::formatName(inner.source_name_, name)}
  {
  }

  static auto formatName(const std::string & inner_source_name, const std::string & name)
    -> std::string
  {
    return inner_source_name.empty() ? name : name + "." + inner_source_name;
  }

  static auto formatName(const std::string & inner_source_name, int index, const std::string & name)
    -> std::string
  {
    std::stringstream ss;
    if (!inner_source_name.empty())
      ss << '.' << name << "[" << std::quoted(inner_source_name) << "]";
    else
      ss << '.' << name << '[' << index << ']';
    return ss.str();
  }

  auto setCoreSource(std::vector<std::pair<std::string, std::string>> conditions_list) -> void
  {
    if (!core_) core_ = std::make_shared<CoreSource>(conditions_list);
  }

  auto setInitActionAsSource() -> void
  {
    if (!core_) core_ = std::make_shared<CoreSource>(true);
  }

  auto recursiveWhat() const -> std::string
  {
    if (inner_) {
      return std::runtime_error::what() + inner_->recursiveWhat();
    } else {
      return std::runtime_error::what();
    }
  }

  const char * what() const noexcept override
  {
    if (core_) {
      core_->logUpdate(recursiveWhat());
      return core_->log().c_str();
    } else
      return "No error core defined";
  }

  std::string source_name_;
  int index_;

private:
  std::shared_ptr<CoreSource> core_;
  const std::shared_ptr<const ScenarioFailure> inner_;
  std::string description_;
};

}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SCENARIO_FAILURE_HPP_
