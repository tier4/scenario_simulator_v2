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

#include <boost/range/adaptor/sliced.hpp>
#include <concealer/autoware.hpp>
#include <exception>

namespace concealer
{
#define AUTOWARE_INFO_STREAM(...) \
  RCLCPP_INFO_STREAM(get_logger(), "\x1b[32m" << __VA_ARGS__ << "\x1b[0m")

#define AUTOWARE_ERROR_STREAM(...) \
  RCLCPP_ERROR_STREAM(get_logger(), "\x1b[1;31m" << __VA_ARGS__ << "\x1b[0m")

#define AUTOWARE_SYSTEM_ERROR(FROM) \
  AUTOWARE_ERROR_STREAM(            \
    "Error on calling " FROM ": " << std::system_error(errno, std::system_category()).what())

Autoware::~Autoware()
{
  AUTOWARE_INFO_STREAM("Shutting down Autoware: (1/3) Stop publlishing/subscribing.");
  {
    if (spinner.joinable()) {
      promise.set_value();
      spinner.join();
    }
  }

  AUTOWARE_INFO_STREAM("Shutting down Autoware: (2/3) Send SIGINT to Autoware launch process.");
  {
    ::kill(process_id, SIGINT);
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

    if (waitpid(process_id, &status, 0) < 0) {
      AUTOWARE_SYSTEM_ERROR("waitpid");
      std::exit(EXIT_FAILURE);
    }
  }
}

void Autoware::update()
{
#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
  setCurrentControlMode();
  setCurrentShift(current_twist);
  setCurrentSteering(current_twist);
  setCurrentTwist(current_twist);
  setCurrentVelocity(current_twist);
  setLocalizationTwist(current_twist);
  setTransform(current_pose);
#endif

#ifdef AUTOWARE_AUTO
  setTransform(current_pose);
#endif
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

/* ---- NOTE -------------------------------------------------------------------
 *
 *  Send initial_pose to Autoware.
 *
 * -------------------------------------------------------------------------- */
void Autoware::initialize(const geometry_msgs::msg::Pose & initial_pose)
{
#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
  task_queue.delay([this, initial_pose]() {
    set(initial_pose);
    waitForAutowareStateToBeInitializingVehicle();
    waitForAutowareStateToBeWaitingForRoute([&]() { setInitialPose(initial_pose); });
  });
#endif

#ifdef AUTOWARE_AUTO
  task_queue.delay(
    [this, initial_pose]() {
      set(initial_pose);
    }
  );
#endif
}

/* ---- NOTE -------------------------------------------------------------------
 *
 *  Send the destination and route constraint points to Autoware. The argument
 *  route is guaranteed to be size 1 or greater, and its last element is the
 *  destination. When the size of a route is 2 or greater, the non-last element
 *  is the route constraint. That is, Autoware must go through the element
 *  points on the given'route' starting at index 0 and stop at index
 *  route.size() - 1.
 *
 * -------------------------------------------------------------------------- */
void Autoware::plan(const std::vector<geometry_msgs::msg::PoseStamped> & route)
{
  assert(0 < route.size());

#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
  task_queue.delay([this, route] {
    waitForAutowareStateToBeWaitingForRoute();  // NOTE: This is assertion.
    setGoalPose(route.back());
    for (const auto & each : route | boost::adaptors::sliced(0, route.size())) {
      setCheckpoint(each);
    }
    waitForAutowareStateToBePlanning();
    waitForAutowareStateToBeWaitingForEngage();  // NOTE: Autoware.IV 0.11.1 waits about 3 sec from the completion of Planning until the transition to WaitingForEngage.
  });
#endif

#ifdef AUTOWARE_AUTO
  task_queue.delay(
    [this, route]() {

    }
  );
#endif
}

/* ---- NOTE -------------------------------------------------------------------
 *
 *  Send an engagement request to Autoware. If Autoware does not have an
 *  engagement equivalent, this operation can be nop (No operation).
 *
 * -------------------------------------------------------------------------- */
void Autoware::engage()
{
#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
  task_queue.delay(
    [this]() { waitForAutowareStateToBeDriving([this]() { setAutowareEngage(true); }); });
#endif

#ifdef AUTOWARE_AUTO
  task_queue.delay(
    [this]() {}
  );
  // TODO (Robotec.ai)
#endif
}
}  // namespace concealer
