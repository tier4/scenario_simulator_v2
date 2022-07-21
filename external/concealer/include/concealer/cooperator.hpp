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

#ifndef CONCEALER__COOPERATOR_HPP_
#define CONCEALER__COOPERATOR_HPP_

#include <boost/lexical_cast.hpp>
#include <concealer/dirty_hack.hpp>
#include <istream>
#include <rclcpp/rclcpp.hpp>
#include <tier4_rtc_msgs/msg/cooperate_status_array.hpp>
#include <tier4_rtc_msgs/srv/cooperate_commands.hpp>
#include <unordered_map>

namespace concealer
{
template <typename Autoware>
struct RTC  // Request To Cooperate
{
  enum class Cooperator {
    simulator,  // DEFAULT
    scenario,
  };

  friend auto operator>>(std::istream & is, Cooperator & cooperator) -> std::istream &
  {
    std::string token;

    is >> token;

    static const std::unordered_map<std::string, Cooperator> table{
      {"simulator", Cooperator::simulator},
      {"scenario", Cooperator::scenario},
    };

    if (auto iter = table.find(token); iter != std::end(table)) {
      cooperator = (*iter).second;
    } else {
      cooperator = Cooperator::simulator;
    }

    return is;
  }

  friend auto operator<<(std::ostream & os, const Cooperator & cooperator) -> std::ostream &
  {
    switch (cooperator) {
      default:
      case Cooperator::simulator:
        return os << "simulator";

      case Cooperator::scenario:
        return os << "scenario";
    }
  }

  Cooperator current_cooperator = Cooperator::simulator;

  using CooperateStatusArray = tier4_rtc_msgs::msg::CooperateStatusArray;

  CONCEALER_DEFINE_SUBSCRIPTION(CooperateStatusArray);

  using CooperateCommands = tier4_rtc_msgs::srv::CooperateCommands;

  CONCEALER_DEFINE_CLIENT_SIMPLE(CooperateCommands);

  explicit RTC()
  : CONCEALER_INIT_SUBSCRIPTION_WITH_CALLBACK(CooperateStatusArray, "/api/external/get/rtc_status", cooperate),
    CONCEALER_INIT_CLIENT(CooperateCommands, "/api/external/set/rtc_commands")
  {
  }

  auto approve(const CooperateStatusArray & cooperate_status_array) -> void
  {
    auto request = std::make_shared<tier4_rtc_msgs::srv::CooperateCommands::Request>();
    request->stamp = cooperate_status_array.stamp;

    auto approvable = [](auto && cooperate_status) {
      return cooperate_status.safe xor
             (cooperate_status.command_status.type == tier4_rtc_msgs::msg::Command::ACTIVATE);
    };

    auto flip = [](auto && type) {
      using Command = tier4_rtc_msgs::msg::Command;
      return type == Command::ACTIVATE ? Command::DEACTIVATE : Command::ACTIVATE;
    };

    for (auto && cooperate_status : cooperate_status_array.statuses) {
      if (approvable(cooperate_status)) {
        tier4_rtc_msgs::msg::CooperateCommand cooperate_command;
        cooperate_command.module = cooperate_status.module;
        cooperate_command.uuid = cooperate_status.uuid;
        cooperate_command.command.type = flip(cooperate_status.command_status.type);
        request->commands.push_back(cooperate_command);
      }
    }

    if (not request->commands.empty()) {
      requestCooperateCommands(request);
    }
  }

  auto cooperate(const CooperateStatusArray & cooperate_status_array) -> void
  {
    switch (current_cooperator) {
      case Cooperator::simulator:
        return approve(cooperate_status_array);

      default:
        return;
    }
  }
};

// auto operator>>(std::istream &, RTC::Cooperator &) -> std::istream &;
//
// auto operator<<(std::ostream &, const RTC::Cooperator &) -> std::ostream &;
}  // namespace concealer

#endif  // CONCEALER__COOPERATOR_HPP_
