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
#include <concealer/autoware_error.hpp>

#define DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(STATE)                                               \
  template <typename Thunk = void (*)(), typename Seconds = std::chrono::seconds>                 \
  void waitForAutowareStateToBe##STATE(                                                           \
    Thunk thunk = []() {}, Seconds interval = std::chrono::seconds(1))                            \
  {                                                                                               \
    static const auto duration_max = std::chrono::seconds(20);                                    \
    Seconds duration{0};                                                                          \
    for (rclcpp::WallRate rate{interval}; not static_cast<Node &>(*this).is##STATE();             \
         rate.sleep()) {                                                                          \
      if ((duration += interval) < duration_max) {                                                \
        RCLCPP_INFO_STREAM(                                                                       \
          static_cast<Node &>(*this).get_logger(),                                                \
          "Simulator waiting for Autoware state to be " #STATE ".");                              \
        thunk();                                                                                  \
      } else {                                                                                    \
        const auto current_state = static_cast<Node &>(*this).getAutowareStatus().autoware_state; \
        std::stringstream what;                                                                   \
        what << "Simulator waited " << duration_max.count()                                       \
             << " seconds, expecting the Autoware state to transitioning to " << #STATE           \
             << ", but there was no change. The current Autoware state is "                       \
             << (current_state.empty() ? "NOT PUBLISHED YET" : current_state)                     \
             << ". This error is most likely due to the Autoware state transition "               \
             << "conditions changing with the update. Please report this error to "               \
             << "the developer. This error message was written by @yamacir-kit.";                 \
        throw AutowareError(what.str());                                                          \
      }                                                                                           \
    }                                                                                             \
    RCLCPP_INFO_STREAM(static_cast<Node &>(*this).get_logger(), "Autoware is " #STATE " now.");   \
  }                                                                                               \
  static_assert(true, "")

namespace concealer
{
template <typename Node>
struct TransitionAssertion
{
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(InitializingVehicle);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(WaitingForRoute);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Planning);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(WaitingForEngage);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Driving);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(ArrivedGoal);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Emergency);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Finalizing);
};

#undef DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE
}  // namespace concealer

#endif  // CONCEALER__TRANSITION_ASSERTION_HPP_
