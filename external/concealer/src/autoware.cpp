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

#include <concealer/autoware.hpp>
#include <exception>

namespace concealer
{
void Autoware::shutdownAutoware()
{
  AUTOWARE_INFO_STREAM("Shutting down Autoware: (1/3) Stop publlishing/subscribing.");
  {
    if (spinner.joinable()) {
      promise.set_value();
      spinner.join();
    }
  }

  if (process_id != 0) {
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
        AUTOWARE_SYSTEM_ERROR("waitpid");
        std::exit(EXIT_FAILURE);
      }
    }
  }
}

void Autoware::rethrow() const
{
  if (thrown) {
    std::rethrow_exception(thrown);
  }
}

bool Autoware::ready() const
{
  task_queue.rethrow();
  rethrow();
  return task_queue.exhausted();
}

void Autoware::resetTimerCallback()
{
  updater = create_wall_timer(std::chrono::milliseconds(5), [this]() { this->update(); });
}

auto Autoware::set(const geometry_msgs::msg::Pose & pose) -> const geometry_msgs::msg::Pose &
{
  return current_pose = pose;
}

auto Autoware::set(const geometry_msgs::msg::Twist & twist) -> const geometry_msgs::msg::Twist &
{
  return current_twist = twist;
}

auto Autoware::setUpperBoundSpeed(double value) -> double  //
{
  return current_upper_bound_speed = std::max(0.0, value);
}
}  // namespace concealer
