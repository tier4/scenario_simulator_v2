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
#include <string>
#include <tier4_simulation_msgs/msg/simulation_events.hpp>
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
class SpecialAction<EXIT_FAILURE> : public std::integral_constant<int, EXIT_FAILURE>,
                                    public std::runtime_error
{
  const std::function<std::string(const std::string &)> make_message;
  const std::string source_name;
  const std::string path;

  template <typename... Ts>
  static auto concatenate(Ts &&... xs)
  {
    std::stringstream ss;
    (ss << ... << xs);
    return ss.str();
  }

  template <typename Container>
  struct comma_separated
  {
    const Container & container;
    const std::function<void(std::ostream &, const typename Container::value_type &)> print;

    template <typename Printer>
    explicit comma_separated(const Container & container, Printer && print)
    : container{container}, print{std::forward<decltype(print)>(print)}
    {
    }

    friend auto operator<<(std::ostream & os, const comma_separated & cs) -> std::ostream &
    {
      auto separator = "";
      for (const auto & value : cs.container) {
        os << std::exchange(separator, ", ");
        cs.print(os, value);
      }
      return os;
    }
  };

  template <typename Conditions>
  static auto makeMessage(
    const std::string & trigger_name, const std::string & trigger_path,
    const Conditions & conditions)
  {
    std::stringstream what;
    what << "CustomCommandAction typed " << std::quoted("exitFailure") << " was triggered by the ";
    const auto anonymous = std::any_of(
      conditions.begin(), conditions.end(),
      [](const auto & condition) { return condition.first.empty(); });
    switch (conditions.size()) {
      case 0:
        what << trigger_name << " (" << trigger_path << ")";
        break;
      case 1:
        if (anonymous) {
          what << "anonymous " << trigger_name << " (" << trigger_path << "." << trigger_name
               << "[0]): " << conditions[0].second;
        } else {
          what << trigger_name << " named " << std::quoted(conditions[0].first) << ": "
               << conditions[0].second;
        }
        break;
      default:
        if (anonymous) {
          what << "anonymous " << trigger_name << "s (" << trigger_path << "." << trigger_name
               << "[...]): "
               << comma_separated(conditions, [i = 0](auto & os, const auto & condition) mutable {
                    os << '{' << std::quoted(std::to_string(i++)) << ": " << condition.second
                       << '}';
                  });
        } else {
          what << "named " << trigger_name << "s {"
               << comma_separated(
                    conditions,
                    [](auto & os, const auto & condition) { os << std::quoted(condition.first); })
               << "}: " << comma_separated(conditions, [](auto & os, const auto & condition) {
                    os << '{' << std::quoted(condition.first) << ": " << condition.second << '}';
                  });
        }
        break;
    }
    return what.str();
  }

public:
  SpecialAction()
  : std::runtime_error{
      "CustomCommandAction typed \"exitFailure\" was triggered on an unexpected code path. This is "
      "a simulator bug. Report this to the simulator developer."}
  {
  }

  explicit SpecialAction(
    const std::function<std::string(const std::string &)> & make_message,
    const std::string & source_name, const std::string & path)
  : std::runtime_error{make_message(path)},
    make_message{make_message},
    source_name{source_name},
    path{path}
  {
  }

  // Constructor with no inner object - first object with conditions
  explicit SpecialAction(
    const std::string & source_name, const std::string & element_name, int element_index,
    const std::string & trigger_name,
    const std::vector<std::pair<std::string, std::string>> & conditions = {})
  : SpecialAction{
      [trigger_name, conditions](const auto & trigger_path) {
        return makeMessage(trigger_name, trigger_path, conditions);
      },
      source_name, concatenate('.', element_name, '[', element_index, ']')}
  {
  }

  // Constructor with inner object and current source (to be forwarded) - intermediate object
  explicit SpecialAction(
    const std::string & source_name, const std::string & element_name, int element_index,
    SpecialAction const & inner)
  : SpecialAction{
      inner.make_message, source_name,
      concatenate(
        '.', element_name, '[',
        (inner.source_name.empty() ? std::to_string(element_index) : '"' + inner.source_name + '"'),
        ']', inner.path)}
  {
  }

  // Constructor with inner but without current source -- last object
  explicit SpecialAction(const std::string & element_name, const SpecialAction & inner)
  : SpecialAction{
      inner.make_message, "",
      concatenate(
        element_name, (inner.source_name.empty() ? "" : "." + inner.source_name), inner.path)}
  {
  }
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
