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

#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/parameter_condition.hpp>
#include <openscenario_interpreter/syntax/parameter_declaration.hpp>
#include <openscenario_interpreter/syntax/user_defined_value_condition.hpp>
#include <regex>

namespace openscenario_interpreter
{
inline namespace syntax
{
template <typename T>
struct MagicSubscription : private rclcpp::Node, public T
{
  std::promise<void> promise;

  std::thread thread;

  std::exception_ptr thrown;

  typename rclcpp::Subscription<T>::SharedPtr subscription;

  explicit MagicSubscription(const std::string & node_name, const std::string & topic_name)
  : rclcpp::Node(node_name),
    thread(
      [this](auto future) {
        while (rclcpp::ok() and
               future.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout) {
          try {
            rclcpp::spin_some(get_node_base_interface());
          } catch (...) {
            thrown = std::current_exception();
          }
        }
      },
      std::move(promise.get_future())),
    subscription(create_subscription<T>(topic_name, 1, [this](const typename T::SharedPtr message) {
      static_cast<T &>(*this) = *message;
    }))
  {
  }
};

UserDefinedValueCondition::UserDefinedValueCondition(const pugi::xml_node & node, Scope & scope)
: name(readAttribute<String>("name", node, scope)),
  value(readAttribute<String>("value", node, scope)),
  rule(readAttribute<Rule>("rule", node, scope))
{
  std::smatch result;

  if (std::regex_match(name, result, std::regex(R"(([^\.]+)\.(.+))"))) {
    //
    const std::unordered_map<std::string, std::function<Element()>> dispatch{
      std::make_pair(
        "currentState", [result]() { return make<String>(evaluateCurrentState(result.str(1))); }),
    };

    evaluateValue = dispatch.at(result.str(2));  // XXX catch

  } else if (std::regex_match(name, result, std::regex(R"(^(?:\/[\w-]+)*\/([\w]+)$)"))) {
    //
    evaluateValue = [&, result]()  //
    {
      static MagicSubscription<openscenario_interpreter_msgs::msg::ParameterDeclaration>
        current_message{"receiver", result.str(0)};

      if (not current_message.value.empty()) {
        const auto parameter_declaration = ParameterDeclaration(current_message);
        PRINT(parameter_declaration.name);
        PRINT(parameter_declaration.parameter_type);
        PRINT(parameter_declaration.value);
        return parameter_declaration.evaluate();
      } else {
        return unspecified;
      }
    };

  } else {
    throw SyntaxError(__FILE__, ":", __LINE__);
  }
}

auto UserDefinedValueCondition::description() const -> String
{
  std::stringstream description;

  description << "Is the " << name << " (= " << result << ") is " << rule << " " << value << "?";

  return description.str();
}

auto UserDefinedValueCondition::evaluate() -> Element
{
  return asBoolean(ParameterCondition::compare(result = evaluateValue(), rule, value));
  // return asBoolean(rule(result = evaluateValue(), value));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
