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

#include <concealer/field_operator_application.hpp>
#include <cstdlib>
#include <exception>
#include <scenario_simulator_exception/exception.hpp>

namespace concealer
{
FieldOperatorApplication::FieldOperatorApplication(const pid_t pid)
: rclcpp::Node("concealer_user", "simulation", rclcpp::NodeOptions().use_global_arguments(false)),
  process_id(pid)
{
}

auto FieldOperatorApplication::stopRequest() noexcept -> void { is_stop_requested.store(true); }

auto FieldOperatorApplication::isStopRequested() const noexcept -> bool
{
  return is_stop_requested.load();
}

auto FieldOperatorApplication::spinSome() -> void
{
  if (rclcpp::ok() and not isStopRequested()) {
    checkAutowareProcess();
    rclcpp::spin_some(get_node_base_interface());
  }
}

auto FieldOperatorApplication::checkAutowareProcess() -> void
{
  if (process_id != 0) {
    int wstatus = 0;
    int ret = waitpid(process_id, &wstatus, WNOHANG);
    if (ret == 0) {
      return;
    } else if (ret < 0) {
      if (errno == ECHILD) {
        is_autoware_exited = true;
        throw common::AutowareError("Autoware process is already terminated");
      } else {
        AUTOWARE_SYSTEM_ERROR("waitpid");
        std::exit(EXIT_FAILURE);
      }
    }

    if (WIFEXITED(wstatus)) {
      is_autoware_exited = true;
      throw common::AutowareError(
        "Autoware process is unintentionally exited. exit code: ", WEXITSTATUS(wstatus));
    } else if (WIFSIGNALED(wstatus)) {
      is_autoware_exited = true;
      throw common::AutowareError("Autoware process is killed. signal is ", WTERMSIG(wstatus));
    }
  }
}

auto FieldOperatorApplication::shutdownAutoware() -> void
{
  AUTOWARE_INFO_STREAM("Shutting down Autoware: (1/3) Stop publishing/subscribing.");
  {
    stopRequest();
  }

  if (process_id != 0 && not is_autoware_exited) {
    is_autoware_exited = true;

    AUTOWARE_INFO_STREAM("Shutting down Autoware: (2/3) Send SIGINT to Autoware launch process.");
    {
      sendSIGINT();
    }

    AUTOWARE_INFO_STREAM("Shutting down Autoware: (2/3) Terminating Autoware.");
    {
      sigset_t mask{};
      {
        sigset_t orig_mask{};

        sigemptyset(&mask);
        sigaddset(&mask, SIGCHLD);

        if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0) {
          AUTOWARE_SYSTEM_ERROR("sigprocmask");
          std::exit(EXIT_FAILURE);
        }
      }

      timespec timeout{};
      {
        timeout.tv_sec = 5;
        timeout.tv_nsec = 0;
      }

      while (sigtimedwait(&mask, NULL, &timeout) < 0) {
        switch (errno) {
          case EINTR:  // Interrupted by a signal other than SIGCHLD.
            break;

          case EAGAIN:
            AUTOWARE_ERROR_STREAM(
              "Shutting down Autoware: (2/3) Autoware launch process does not respond. Kill it.");
            kill(process_id, SIGKILL);
            break;

          default:
            AUTOWARE_SYSTEM_ERROR("sigtimedwait");
            std::exit(EXIT_FAILURE);
        }
      }
    }

    AUTOWARE_INFO_STREAM("Shutting down Autoware: (3/3) Waiting for Autoware to be exited.");
    {
      int status = 0;

      const int waitpid_options = 0;

      if (waitpid(process_id, &status, waitpid_options) < 0) {
        if (errno == ECHILD) {
          AUTOWARE_WARN_STREAM("Try to wait for the autoware process but it was already exited.");
        } else {
          AUTOWARE_SYSTEM_ERROR("waitpid");
          std::exit(EXIT_FAILURE);
        }
      }
    }
  }
}

auto FieldOperatorApplication::getTurnIndicatorsCommand() const
  -> tier4_external_api_msgs::msg::TurnSignalStamped
{
  static auto turn_indicators_command = []() {
    tier4_external_api_msgs::msg::TurnSignalStamped turn_indicators_command;
    turn_indicators_command.turn_signal.data =
      tier4_external_api_msgs::msg::TurnSignal::NONE;
    return turn_indicators_command;
  }();
  turn_indicators_command.stamp = now();
  return turn_indicators_command;
}

auto FieldOperatorApplication::rethrow() const -> void { task_queue.rethrow(); }
}  // namespace concealer

namespace tier4_external_api_msgs::msg
{
auto operator<<(
  std::ostream & out, const TurnSignalStamped & message)
  -> std::ostream &
{
#define CASE(IDENTIFIER)                                                   \
  case TurnSignal::IDENTIFIER: \
    out << #IDENTIFIER;                                                    \
    break

  switch (message.turn_signal.data) {
    CASE(NONE);
    CASE(LEFT);
    CASE(RIGHT);
    CASE(HAZARD);

    default:
      throw common::Error(
        "Unsupported TurnIndicatorsCommand, state number : ", static_cast<int>(message.turn_signal.data));
  }

#undef CASE

  return out;
}

auto operator>>(std::istream & is, TurnSignalStamped & message)
  -> std::istream &
{
#define STATE(IDENTIFIER) \
  {#IDENTIFIER, TurnSignal::IDENTIFIER}

  std::unordered_map<std::string, std::uint8_t> state_dictionary{
    STATE(NONE),
    STATE(LEFT),
    STATE(RIGHT),
    STATE(HAZARD),
  };

#undef STATE

  std::string data_string;
  is >> data_string;

  if (auto iter = state_dictionary.find(data_string); iter != state_dictionary.end()) {
    message.turn_signal.set__data(iter->second);
  } else {
    throw common::Error("Unsupported TurnSignal::data : ", data_string.c_str());
  }

  return is;
}
}  // namespace tier4_external_api_msgs::msg
