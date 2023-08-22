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

#include <exception>
#include <iomanip>
#include <scenario_simulator_exception/exception.hpp>

namespace openscenario_interpreter
{
struct ScenarioFailure : public std::runtime_error
{
private:
  struct CoreSource
  {
    CoreSource() = default;

    explicit CoreSource(const std::vector<std::pair<std::string, std::string>> & conditions_list)
    : single_{conditions_list.size() == 1}
    {
      anonymous_ = std::any_of(
        conditions_list.begin(), conditions_list.end(),
        [](const auto & pair) { return pair.first.empty(); });

      std::stringstream element_name(""), description("");
      if (single_) {
        element_name << std::quoted(conditions_list.front().first);
        description << conditions_list.front().second;
      } else {
        for (size_t i = 0; const auto & condition : conditions_list) {
          element_name << std::quoted(condition.first);
          description << "{" << std::quoted(anonymous_ ? std::to_string(i++) : condition.first)
                      << ": " << condition.second << "}";
          if (&condition != &conditions_list.back()) {
            element_name << ", ";
            description << ", ";
          }
        }
      }
      element_name_ = element_name.str();
      description_ = description.str();
    }

    auto log() const -> const std::string & { return log_; }

    auto logUpdate(const std::string & trigger_path) -> std::string
    {
      std::stringstream log;
      log << "CustomCommandAction typed " << std::quoted("exitFailure") << " was triggered by the ";
      if (element_name_.empty()) {
        log << "action (" << trigger_path << ")";
      } else if (single_ && anonymous_) {
        log << "anonymous condition (" << trigger_path << ".Condition[0]): " << description_;
      } else if (single_) {
        log << "Condition named " << element_name_ << ": " << description_;
      } else if (anonymous_) {
        log << "anonymous conditions (" << trigger_path << ".Condition[...]): " << description_;
      } else {
        log << "Conditions named {" << element_name_ << "}: " << description_;
      }
      log_ = log.str();
      return log_;
    }

  private:
    bool anonymous_{false}, single_{true}, without_condition_{false};
    std::string element_name_{""};
    std::string description_{""};
    std::string log_{""};
  };

public:
  ScenarioFailure() : std::runtime_error{""} {};
  ~ScenarioFailure() = default;
  ScenarioFailure(const ScenarioFailure &) = default;
  ScenarioFailure & operator=(const ScenarioFailure &) = default;
  ScenarioFailure(ScenarioFailure &&) = default;
  ScenarioFailure & operator=(ScenarioFailure &&) = default;

  // Constructor with no inner object -- first object
  ScenarioFailure(
    const std::string & source_name, int element_index, const std::string & element_name)
  : source_name_{source_name},
    inner_{nullptr},
    core_{nullptr},
    std::runtime_error{ScenarioFailure::formatSubpath("", element_index, element_name)}
  {
  }

  // Constructor with inner object and current source (to be forwarded) -- intermediate object
  ScenarioFailure(
    const std::string & source_name, int element_index, const std::string & element_name,
    ScenarioFailure const & inner)
  : source_name_{source_name},
    inner_{std::make_shared<ScenarioFailure>(inner)},
    core_{inner.core_},
    std::runtime_error{
      ScenarioFailure::formatSubpath(inner.source_name_, element_index, element_name)}
  {
  }

  // Constructor with inner but without current source -- last object
  ScenarioFailure(const std::string & element_name, const ScenarioFailure & inner)
  : source_name_{""},
    inner_{std::make_shared<ScenarioFailure>(inner)},
    core_{inner.core_},
    std::runtime_error{ScenarioFailure::formatSubpath(inner.source_name_, element_name)}
  {
  }

  auto setCoreSource(const std::vector<std::pair<std::string, std::string>> & conditions_list)
    -> void
  {
    if (!core_)
      core_ = std::make_shared<CoreSource>(conditions_list);
    else
      throw Error("The scenario result core has already been instantiated.");
  }

  auto setInitActionAsSource() -> void
  {
    if (!core_)
      core_ = std::make_shared<CoreSource>();
    else
      throw Error("The scenario result core has already been instantiated.");
  }

  const char * what() const noexcept override
  {
    if (core_) {
      core_->logUpdate(recursiveWhat());
      return core_->log().c_str();
    }
    return "No core defined";
  }

private:
  auto recursiveWhat() const -> std::string
  {
    if (inner_) return std::runtime_error::what() + inner_->recursiveWhat();
    return std::runtime_error::what();
  }

  static auto formatSubpath(const std::string & inner_source_name, const std::string & element_name)
    -> std::string
  {
    return inner_source_name.empty() ? element_name : element_name + "." + inner_source_name;
  }

  static auto formatSubpath(
    const std::string & inner_source_name, int element_index, const std::string & element_name)
    -> std::string
  {
    std::stringstream ss("");
    ss << '.' << element_name;
    if (!inner_source_name.empty())
      ss << "[" << std::quoted(inner_source_name) << "]";
    else
      ss << '[' << element_index << ']';
    return ss.str();
  }

  std::shared_ptr<CoreSource> core_{nullptr};
  const std::shared_ptr<const ScenarioFailure> inner_{nullptr};
  const std::string source_name_{""};
};

}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__SCENARIO_FAILURE_HPP_
