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

#include <chrono>
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
  const std::chrono::seconds given;

  std::chrono::seconds remains;

  explicit TransitionAssertion() : given(getParameter<int>("initialize_duration")), remains(given)
  {
  }

  // auto makeTransitionError(const std::string & expected) const
  // {
  //   const auto current_state = static_cast<const Autoware &>(*this).getAutowareStatus().autoware_state;
  //   return common::AutowareError(
  //     "Simulator waited ", given.count(),
  //     " seconds, expecting the Autoware state to transitioning to ", expected,
  //     ", but there was no change. The current Autoware state is ",
  //     (current_state.empty() ? "NOT PUBLISHED YET" : current_state), ".");
  // }

#define DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(STATE)                                             \
  template <typename Thunk = void (*)()>                                                        \
  void waitForAutowareStateToBe##STATE(                                                         \
    Thunk thunk = []() {}, const std::chrono::seconds & interval = std::chrono::seconds(1))     \
  {                                                                                             \
    rclcpp::WallRate rate{interval};                                                            \
    for (thunk(); static_cast<Autoware &>(*this).currentFuture().wait_for(                      \
                    std::chrono::milliseconds(1)) == std::future_status::timeout and            \
                  not static_cast<const Autoware &>(*this).is##STATE();                         \
         rate.sleep()) {                                                                        \
      remains -= std::chrono::duration_cast<std::chrono::seconds>(rate.period());               \
      RCLCPP_INFO_STREAM(                                                                       \
        static_cast<Autoware &>(*this).get_logger(),                                            \
        "Simulator waiting for Autoware state to be " #STATE " (" << remains.count() << ").");  \
      thunk();                                                                                  \
    }                                                                                           \
    RCLCPP_INFO_STREAM(                                                                         \
      static_cast<Autoware &>(*this).get_logger(),                                              \
      "Autoware is " << static_cast<const Autoware &>(*this).getAutowareStatus().autoware_state \
                     << " now.");                                                               \
  }                                                                                             \
  static_assert(true, "")

  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(InitializingVehicle);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(WaitingForRoute);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Planning);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(WaitingForEngage);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Driving);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(ArrivedGoal);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Emergency);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Finalizing);

#undef DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE
};
}  // namespace concealer

#endif  // CONCEALER__TRANSITION_ASSERTION_HPP_
