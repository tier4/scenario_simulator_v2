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

#ifndef CONCEALER__TRANSITION_ASSERTION_HPP_
#define CONCEALER__TRANSITION_ASSERTION_HPP_

#include <chrono>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <scenario_simulator_exception/exception.hpp>

namespace concealer
{
template <typename Autoware>
struct TransitionAssertion
{
protected:
  const std::chrono::steady_clock::time_point start;

  const std::chrono::seconds initialize_duration;

  bool have_never_been_engaged = true;

  explicit TransitionAssertion()
  : start(std::chrono::steady_clock::now()), initialize_duration([]() {
      auto node = rclcpp::Node("get_parameter", "simulation");
      node.declare_parameter("initialize_duration", 0);
      return node.get_parameter("initialize_duration").as_int();
    }())
  {
  }

  template <typename Thunk = void (*)(), typename Interval = std::chrono::seconds>
  auto waitForAutowareStateToBe(
    const std::string & state, Thunk thunk = [] {}, Interval interval = std::chrono::seconds(1))
  {
    for (thunk(); not static_cast<const Autoware &>(*this).is_stop_requested.load() and
                  static_cast<const Autoware &>(*this).autoware_state != state;
         rclcpp::GenericRate<std::chrono::steady_clock>(interval).sleep()) {
      if (
        have_never_been_engaged and
        start + initialize_duration <= std::chrono::steady_clock::now()) {
        const auto state = static_cast<const Autoware &>(*this).autoware_state;
        throw common::AutowareError(
          "Simulator waited for the Autoware state to transition to ", state,
          ", but time is up. The current Autoware state is ",
          (state.empty() ? "not published yet" : state));
      } else {
        thunk();
      }
    }

    if (state == "DRIVING") {
      have_never_been_engaged = false;
    }
  }
};
}  // namespace concealer

#endif  // CONCEALER__TRANSITION_ASSERTION_HPP_
