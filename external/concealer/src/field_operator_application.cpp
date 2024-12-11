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
#include <system_error>

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
    if (process_id != 0) {
      auto status = 0;
      if (const auto id = waitpid(process_id, &status, WNOHANG); id < 0) {
        switch (errno) {
          case ECHILD:
            is_autoware_exited = true;
            throw common::AutowareError("Autoware process is already terminated");
          default:
            AUTOWARE_SYSTEM_ERROR("waitpid");
            std::exit(EXIT_FAILURE);
        }
      } else if (0 < id) {
        if (WIFEXITED(status)) {
          is_autoware_exited = true;
          throw common::AutowareError(
            "Autoware process is unintentionally exited. exit code: ", WEXITSTATUS(status));
        } else if (WIFSIGNALED(status)) {
          is_autoware_exited = true;
          throw common::AutowareError("Autoware process is killed. signal is ", WTERMSIG(status));
        }
      }
    }
    rclcpp::spin_some(get_node_base_interface());
  }
}

auto FieldOperatorApplication::shutdownAutoware() -> void
{
  if (stopRequest(); process_id != 0 && not std::exchange(is_autoware_exited, true)) {
    const auto sigset = [this]() {
      if (auto signal_set = sigset_t();
          sigemptyset(&signal_set) or sigaddset(&signal_set, SIGCHLD)) {
        RCLCPP_ERROR_STREAM(get_logger(), std::system_error(errno, std::system_category()).what());
        std::exit(EXIT_FAILURE);
      } else if (auto error = pthread_sigmask(SIG_BLOCK, &signal_set, nullptr)) {
        RCLCPP_ERROR_STREAM(get_logger(), std::system_error(error, std::system_category()).what());
        std::exit(EXIT_FAILURE);
      } else {
        return signal_set;
      }
    }();

    const auto timeout = []() {
      auto sigterm_timeout = [](auto value) {
        auto node = rclcpp::Node("get_parameter_sigterm_timeout", "simulation");
        node.declare_parameter<int>("sigterm_timeout", value);
        node.get_parameter<int>("sigterm_timeout", value);
        return value;
      };
      auto timeout = timespec();
      timeout.tv_sec = sigterm_timeout(5);
      timeout.tv_nsec = 0;
      return timeout;
    }();

    if (::kill(process_id, SIGINT); sigtimedwait(&sigset, nullptr, &timeout) < 0) {
      switch (errno) {
        case EINTR:
          /*
             The wait was interrupted by an unblocked, caught signal. It shall
             be documented in system documentation whether this error causes
             these functions to fail.
          */
          RCLCPP_ERROR_STREAM(
            get_logger(),
            "The wait for Autoware launch process termination was interrupted by an unblocked, "
            "caught signal.");
          break;

        case EAGAIN:
          /*
             No signal specified by set was generated within the specified
             timeout period.
          */
          RCLCPP_ERROR_STREAM(get_logger(), "Autoware launch process does not respond. Kill it.");
          killpg(process_id, SIGKILL);
          break;

        default:
        case EINVAL:
          /*
             The timeout argument specified a tv_nsec value less than zero or
             greater than or equal to 1000 million.
          */
          RCLCPP_ERROR_STREAM(
            get_logger(),
            "The parameter sigterm_timeout specified a value less than zero or greater than or "
            "equal to 1000 million.");
          break;
      }
    }

    if (int status = 0; waitpid(process_id, &status, 0) < 0) {
      if (errno == ECHILD) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Try to wait for the autoware process but it was already exited.");
      } else {
        RCLCPP_ERROR_STREAM(get_logger(), std::system_error(errno, std::system_category()).what());
      }
    }
  }
}

auto FieldOperatorApplication::rethrow() const -> void { task_queue.rethrow(); }
}  // namespace concealer
