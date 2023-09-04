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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <pugixml.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_simulation_msgs/msg/simulation_events.hpp>

#include <string>
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
template <int Value>
struct SpecialAction : public std::integral_constant<int, Value>
{
};

template <>
struct SpecialAction<EXIT_FAILURE> : public std::integral_constant<int, EXIT_FAILURE>
{
private:
  struct CoreSource
  {
    enum class SourceType {
      UNKNOWN,
      ANONYMOUS,
      ANONYMOUS_CONDITION,
      ANONYMOUS_CONDITIONS,
      NAMED_CONDITION,
      NAMED_CONDITIONS
    };

    explicit CoreSource(const std::string & trigger_name)
    : trigger_name_{trigger_name},
      type_{SourceType::ANONYMOUS},
      conditions_description_{""},
      conditions_name_{""}
    {
    }

    CoreSource(
      const std::string & trigger_name,
      const std::vector<std::pair<std::string, std::string>> & conditions_list)
    : trigger_name_{trigger_name}
    {
      auto anonymous{std::any_of(
        conditions_list.begin(), conditions_list.end(),
        [](const auto & condition) { return condition.first.empty(); })};

      std::stringstream conditions_name(""), conditions_description("");
      if (conditions_list.size() == 1) {
        type_ = anonymous ? SourceType::ANONYMOUS_CONDITION : SourceType::NAMED_CONDITION;
        conditions_name << std::quoted(conditions_list.front().first);
        conditions_description << conditions_list.front().second;
      } else {
        type_ = anonymous ? SourceType::ANONYMOUS_CONDITIONS : SourceType::NAMED_CONDITIONS;
        std::size_t i = 0;
        for (const auto & condition : conditions_list) {
          conditions_name << std::quoted(condition.first);
          conditions_description << "{"
                                 << std::quoted(anonymous ? std::to_string(i++) : condition.first)
                                 << ": " << condition.second << "}";
          if (&condition != &conditions_list.back()) {
            conditions_name << ", ";
            conditions_description << ", ";
          }
        }
      }
      conditions_name_ = conditions_name.str();
      conditions_description_ = conditions_description.str();
    }

    auto comprehensiveLog(const std::string & trigger_path) -> std::string
    {
      std::stringstream log;
      log << "CustomCommandAction typed " << std::quoted("exitFailure") << " was triggered by the ";
      switch (type_) {
        case SourceType::ANONYMOUS: {
          log << trigger_name_ << " (" << trigger_path << ")";
          break;
        }
        case SourceType::ANONYMOUS_CONDITION: {
          log << "anonymous " << trigger_name_ << " (" << trigger_path << "." << trigger_name_
              << "[0]): " << conditions_description_;
          break;
        }
        case SourceType::NAMED_CONDITION: {
          log << trigger_name_ << " named " << conditions_name_ << " : " << conditions_description_;
          break;
        }
        case SourceType::ANONYMOUS_CONDITIONS: {
          log << "anonymous " << trigger_name_ << "s (" << trigger_path << "." << trigger_name_
              << "[...]): " << conditions_description_;
          break;
        }
        case SourceType::NAMED_CONDITIONS: {
          log << "named " << trigger_name_ << "s {" << conditions_name_
              << "}: " << conditions_description_;
          break;
        }
        default: {
          throw std::runtime_error(
            "Unsupported case of CoreSource in SpecialAction - this should not happen.");
        }
      }
      return log.str();
    }

  private:
    const std::string trigger_name_{""};
    SourceType type_{SourceType::UNKNOWN};
    std::string conditions_description_{""};
    std::string conditions_name_{""};
  };

public:
  SpecialAction() = default;
  ~SpecialAction() = default;
  SpecialAction(const SpecialAction &) = default;
  SpecialAction & operator=(const SpecialAction &) = default;
  SpecialAction(SpecialAction &&) = default;
  SpecialAction & operator=(SpecialAction &&) = default;

  // Constructor with no inner object - first object with conditions_list
  SpecialAction(
    const std::string & source_name, int element_index, const std::string & element_name,
    const std::string & trigger_name_,
    const std::vector<std::pair<std::string, std::string>> & conditions_list)
  : source_name_{source_name},
    inner_{nullptr},
    core_{std::make_shared<CoreSource>(trigger_name_, conditions_list)},
    path_{formatPath("", element_index, element_name)},
    log_{core_->comprehensiveLog(path_)}
  {
  }

  // Constructor with no inner object - first object without conditions
  SpecialAction(
    const std::string & source_name, int element_index, const std::string & element_name,
    const std::string & trigger_name_)
  : source_name_{source_name},
    inner_{nullptr},
    core_{std::make_shared<CoreSource>(trigger_name_)},
    path_{formatPath("", element_index, element_name)},
    log_{core_->comprehensiveLog(path_)}
  {
  }

  // Constructor with inner object and current source (to be forwarded) - intermediate object
  SpecialAction(
    const std::string & source_name, int element_index, const std::string & element_name,
    SpecialAction const & inner)
  : source_name_{source_name},
    inner_{std::make_shared<SpecialAction>(inner)},
    core_{inner.core_},
    path_{formatPath(inner.source_name_, element_index, element_name)},
    log_{core_->comprehensiveLog(path_)}
  {
  }

  // Constructor with inner but without current source -- last object
  SpecialAction(const std::string & element_name, const SpecialAction & inner)
  : source_name_{""},
    inner_{std::make_shared<SpecialAction>(inner)},
    core_{inner.core_},
    path_{formatPath(inner.source_name_, element_name)},
    log_{core_->comprehensiveLog(path_)}
  {
  }

  const char * what() const noexcept { return log_.c_str(); }

private:
  auto formatPath(const std::string & inner_source_name, const std::string & element_name)
    -> std::string
  {
    if (inner_) {
      return (inner_source_name.empty() ? element_name : element_name + "." + inner_source_name) +
             inner_->path_;
    } else {
      return inner_source_name.empty() ? element_name : element_name + "." + inner_source_name;
    }
  }

  auto formatPath(
    const std::string & inner_source_name, int element_index, const std::string & element_name)
    -> std::string
  {
    std::stringstream ss("");
    ss << '.' << element_name;
    if (!inner_source_name.empty()) {
      ss << "[" << std::quoted(inner_source_name) << "]";
    } else {
      ss << '[' << element_index << ']';
    }
    if (inner_) {
      ss << inner_->path_;
    }
    return ss.str();
  }

  const std::shared_ptr<const SpecialAction> inner_{nullptr};
  std::shared_ptr<CoreSource> core_{nullptr};
  const std::string source_name_{""};
  const std::string path_{""};
  const std::string log_{""};
};

struct CustomCommand
{
  const std::vector<std::string> parameters;

  CustomCommand() = default;

  CustomCommand(const CustomCommand &) = default;

  CustomCommand(CustomCommand &&) = default;

  explicit CustomCommand(const std::vector<std::string> & parameters) : parameters(parameters) {}

  virtual ~CustomCommand() = default;

  virtual auto accomplished() noexcept -> bool { return true; }

  virtual auto endsImmediately() const -> bool { return true; }

  virtual auto run() noexcept -> void {}

  virtual auto start(const Scope &) -> void {}
};

/* ---- CustomCommandAction ----------------------------------------------------
 *
 *  <xsd:complexType name="CustomCommandAction">
 *    <xsd:simpleContent>
 *      <xsd:extension base="xsd:string">
 *        <xsd:attribute name="type" type="String" use="required"/>
 *      </xsd:extension>
 *    </xsd:simpleContent>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct CustomCommandAction : private Scope
{
  const String type;

  const String content;

private:
  const std::shared_ptr<CustomCommand> command;

public:
  explicit CustomCommandAction(const pugi::xml_node &, const Scope &);

  auto accomplished() noexcept -> bool { return command->accomplished(); }

  auto endsImmediately() const -> bool { return command->endsImmediately(); }

  auto run() noexcept -> void { return command->run(); }

  auto start() -> void { command->start(local()); }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_
