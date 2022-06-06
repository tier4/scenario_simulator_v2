// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef CONCEALER__TRANSITION_ASSERTION_HPP_
#define CONCEALER__TRANSITION_ASSERTION_HPP_

#include <autoware_auto_system_msgs/msg/autoware_state.hpp>
#include <chrono>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <scenario_simulator_exception/exception.hpp>

namespace concealer
{
template <typename T>
auto getParameter(const std::string & name, T value = {})
{
  rclcpp::Node node{"get_parameter", "simulation"};

  node.declare_parameter<T>(name, value);
  node.get_parameter<T>(name, value);

  return value;
}

template <typename Autoware>
struct TransitionAssertion
{
  using clock_type = std::chrono::steady_clock;
  using rate_type = rclcpp::GenericRate<clock_type>;

  const clock_type::time_point start;
  const std::chrono::seconds initialize_duration;

  explicit TransitionAssertion()
  : start(clock_type::now()),
    initialize_duration(std::chrono::seconds(getParameter<int>("initialize_duration")))
  {
  }

  auto makeTransitionError(const std::string & expected) const
  {
    const auto current_state = asAutoware().getAutowareStateName();
    return common::AutowareError(
      "Simulator waited for the Autoware state to transition to ", expected,
      ", but time is up. The current Autoware state is ",
      (current_state.empty() ? "NOT PUBLISHED YET" : current_state), ".");
  }

  auto asAutoware() -> Autoware & { return static_cast<Autoware &>(*this); }
  auto asAutoware() const -> const Autoware & { return static_cast<const Autoware &>(*this); }

#define DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(STATE)                                        \
  template <typename Thunk = void (*)()>                                                   \
  void waitForAutowareStateToBe##STATE(                                                    \
    Thunk && thunk = [] {}, std::chrono::seconds interval = std::chrono::seconds(1))       \
  {                                                                                        \
    using namespace std::chrono;                                                           \
    rate_type rate(interval);                                                              \
    for (thunk(); not asAutoware().isStopRequested() and not asAutoware().is##STATE();     \
         rate.sleep(), thunk()) {                                                          \
      if (auto now = clock_type::now(); start + initialize_duration <= now) {              \
        throw makeTransitionError(#STATE);                                                 \
      } else {                                                                             \
        RCLCPP_INFO_STREAM(                                                                \
          asAutoware().get_logger(),                                                       \
          "Simulator waiting for Autoware state to be " #STATE " (remain: "                \
            << duration_cast<seconds>(start + initialize_duration - now).count() << ")."); \
      }                                                                                    \
    }                                                                                      \
    RCLCPP_INFO_STREAM(                                                                    \
      asAutoware().get_logger(),                                                           \
      "Autoware is " << asAutoware().getAutowareStateName() << " now.");                   \
  }                                                                                        \
  static_assert(true, "")

  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Initializing);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(WaitingForRoute);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Planning);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(WaitingForEngage);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Driving);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(ArrivedGoal);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Finalizing);

#undef DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE
};
}  // namespace concealer

#endif  // CONCEALER__TRANSITION_ASSERTION_HPP_
