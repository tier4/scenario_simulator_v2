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

#include <boost/lexical_cast.hpp>
#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/functional/curry.hpp>
#include <openscenario_interpreter/regex/function_call_expression.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/lane_position.hpp>        // for RelativeHeadingCondition
#include <openscenario_interpreter/syntax/parameter_condition.hpp>  // for ParameterCondition::compare
#include <openscenario_interpreter/syntax/parameter_declaration.hpp>
#include <openscenario_interpreter/syntax/user_defined_value_condition.hpp>
#include <regex>
#include <unordered_map>

#if __has_include(<tier4_simulation_msgs/msg/user_defined_value.hpp>)
#include <tier4_simulation_msgs/msg/user_defined_value.hpp>
#endif

namespace openscenario_interpreter
{
inline namespace syntax
{
template <typename T>
struct MagicSubscription : private T
{
  struct Node : public rclcpp::Node
  {
    std::atomic_bool stop_requested = false;

    std::mutex mutex;

    std::exception_ptr thrown;

    std::thread thread;

    explicit Node()
    : rclcpp::Node("magic_subscription"), thread([this]() {
        while (rclcpp::ok() and not stop_requested) {
          try {
            rclcpp::spin_some(get_node_base_interface());
          } catch (...) {
            auto lock = std::lock_guard(mutex);
            thrown = std::current_exception();
          }
        }
      })
    {
    }

    ~Node()
    {
      if (thread.joinable() and not stop_requested.exchange(true)) {
        thread.join();
      }
    }

    auto rethrow()
    {
      if (auto lock = std::lock_guard(mutex); thrown) {
        std::rethrow_exception(thrown);
      }
    }
  };

  static inline std::size_t count = 0;

  static inline std::unique_ptr<Node> node = nullptr;

  typename rclcpp::Subscription<T>::SharedPtr subscription;

  explicit MagicSubscription(const std::string & topic_name)
  {
    if (not count++) {
      node = std::make_unique<Node>();
    }

    subscription = node->template create_subscription<T>(
      topic_name, rclcpp::QoS(1).best_effort(),
      [this](const typename T::SharedPtr message) { static_cast<T &>(*this) = *message; });
  }

  ~MagicSubscription()
  {
    if (not --count) {
      node.reset();
    }
  }

  auto get() const -> const auto &
  {
    node->rethrow();
    return static_cast<const T &>(*this);
  }
};

UserDefinedValueCondition::UserDefinedValueCondition(const pugi::xml_node & node, Scope & scope)
: name(readAttribute<String>("name", node, scope)),
  value(readAttribute<String>("value", node, scope)),
  rule(readAttribute<Rule>("rule", node, scope))
{
  if (std::smatch result; std::regex_match(name, result, std::regex(R"(([^.]+)\.(.+))"))) {
    const std::unordered_map<std::string, std::function<Object()>> dispatch{
      std::make_pair(
        "currentState", [result]() { return make<String>(evaluateCurrentState(result.str(1))); }),
      std::make_pair(
        "currentMinimumRiskManeuverState.behavior",
        [result]() {
          return make<String>(
            asFieldOperatorApplication(result.str(1)).getMinimumRiskManeuverBehaviorName());
        }),
      std::make_pair(
        "currentMinimumRiskManeuverState.state",
        [result]() {
          return make<String>(
            asFieldOperatorApplication(result.str(1)).getMinimumRiskManeuverStateName());
        }),
      std::make_pair(
        "currentEmergencyState",
        [result]() {
          return make<String>(asFieldOperatorApplication(result.str(1)).getEmergencyStateName());
        }),
      std::make_pair(
        "currentTurnIndicatorsState",
        [result]() {
          return make<String>(boost::lexical_cast<String>(
            asFieldOperatorApplication(result.str(1)).getTurnIndicatorsCommand()));
        }),
    };
    evaluate_value = dispatch.at(result.str(2));  // XXX catch
  } else if (std::regex_match(name, result, FunctionCallExpression::pattern())) {
    const std::unordered_map<std::string, std::function<Object(const std::vector<std::string> &)>>
      functions{
        std::make_pair(
          "RelativeHeadingCondition",
          [this, result](const auto & xs) {
            switch (std::size(xs)) {
              case 1:  // RelativeHeadingCondition(<ENTITY-REF>)
                return make<Double>(evaluateRelativeHeading(xs[0]));
              case 3:  // RelativeHeadingCondition(<ENTITY-REF>, <LANE-ID>, <S>)
                return make<Double>(evaluateRelativeHeading(
                  xs[0], LanePosition("", xs[1], 0, boost::lexical_cast<Double>(xs[2]))));
              default:
                return make<Double>(Double::nan());
            }
          }),
      };
    evaluate_value =
      curry2(functions.at(result.str(1)))(FunctionCallExpression::splitParameters(result.str(3)));
  } else if (std::regex_match(name, result, std::regex(R"(^(?:\/[\w-]+)*\/([\w]+)$)"))) {
#if __has_include(<tier4_simulation_msgs/msg/user_defined_value.hpp>)
    using tier4_simulation_msgs::msg::UserDefinedValue;
    using tier4_simulation_msgs::msg::UserDefinedValueType;

    evaluate_value =
      [this, subscriber = std::make_shared<MagicSubscription<UserDefinedValue>>(result.str(0))]() {
        if (const auto message = subscriber->get(); message.value.empty()) {
          return unspecified;
        } else {
          switch (message.type.data) {
            case UserDefinedValueType::BOOLEAN:
              return make<Boolean>(message.value);
            case UserDefinedValueType::DATE_TIME:
              return make<String>(message.value);
            case UserDefinedValueType::DOUBLE:
              return make<Double>(message.value);
            case UserDefinedValueType::INTEGER:
              return make<Integer>(message.value);
            case UserDefinedValueType::STRING:
              return make<String>(message.value);
            case UserDefinedValueType::UNSIGNED_INT:
              return make<UnsignedInt>(message.value);
            case UserDefinedValueType::UNSIGNED_SHORT:
              return make<UnsignedShort>(message.value);
            default:
              return unspecified;
          }
        }
      };
#else
    throw SyntaxError(
      "The ability to have ROS 2 topics as values for `UserDefinedValueCondition` is enabled only "
      "when the `UserDefinedValue` type is present in the `tier4_simulation_msgs` package.");
#endif
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

auto UserDefinedValueCondition::evaluate() -> Object
{
  if (result = evaluate_value(); result == unspecified) {
    return false_v;
  } else {
    return asBoolean(ParameterCondition::compare(result, rule, value));
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
