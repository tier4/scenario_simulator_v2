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

#include <concealer/autoware.hpp>
#include <cstdlib>
#include <exception>
#include <scenario_simulator_exception/exception.hpp>

namespace concealer
{
void Autoware::checkAutowareProcess()
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

auto Autoware::getGearCommand() const -> autoware_auto_vehicle_msgs::msg::GearCommand
{
  static auto gear_command = []() {
    autoware_auto_vehicle_msgs::msg::GearCommand gear_command;
    gear_command.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE;
    return gear_command;
  }();
  gear_command.stamp = now();
  return gear_command;
}

auto Autoware::getTurnIndicatorsCommand() const
  -> autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand
{
  static auto turn_indicators_command = []() {
    autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand turn_indicators_command;
    turn_indicators_command.command =
      autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND;
    return turn_indicators_command;
  }();
  turn_indicators_command.stamp = now();
  return turn_indicators_command;
}

void Autoware::shutdownAutoware()
{
  AUTOWARE_INFO_STREAM("Shutting down Autoware: (1/3) Stop publishing/subscribing.");
  {
    if (spinner.joinable()) {
      stopRequest();
      spinner.join();
    }
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

void Autoware::rethrow() const
{
  task_queue.rethrow();

  if (is_thrown.load(std::memory_order_acquire)) {
    std::rethrow_exception(thrown);
  }
}

void Autoware::resetTimerCallback()
{
  updater = create_wall_timer(std::chrono::milliseconds(5), [this]() { this->update(); });
}

auto Autoware::set(const geometry_msgs::msg::Accel & acceleration)
  -> const geometry_msgs::msg::Accel &
{
  return current_acceleration = acceleration;
}

auto Autoware::set(const geometry_msgs::msg::Pose & pose) -> const geometry_msgs::msg::Pose &
{
  return current_pose = pose;
}

auto Autoware::set(const geometry_msgs::msg::Twist & twist) -> const geometry_msgs::msg::Twist &
{
  return current_twist = twist;
}
}  // namespace concealer
