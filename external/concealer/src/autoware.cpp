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

#define DEBUG_VALUE(...) \
  std::cout << "\x1b[32m" #__VA_ARGS__ " = " << (__VA_ARGS__) << "\x1b[0m" << std::endl

#define DEBUG_LINE() \
  std::cout << "\x1b[32m" << __FILE__ << ":" << __LINE__ << "\x1b[0m" << std::endl

namespace concealer
{
Autoware::~Autoware()
{
  if (spinner.joinable()) {
    promise.set_value();
    spinner.join();
  }

  int status = 0;

  if (::kill(process_id, SIGINT) < 0 or ::waitpid(process_id, &status, WUNTRACED) < 0) {
    std::cout << std::system_error(errno, std::system_category()).what() << std::endl;
    std::exit(EXIT_FAILURE);
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
  // TODO (Robotec.ai)
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
  task_queue.delay([&]() {
    set(initial_pose);
    waitForAutowareStateToBeInitializingVehicle();
    waitForAutowareStateToBeWaitingForRoute([&]() { setInitialPose(initial_pose); });
  });
#endif

#ifdef AUTOWARE_AUTO
  task_queue.delay([&]() {
    // TODO (Robotec.ai)
  });
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

#ifndef AUTOWARE_AUTO
  // TODO (Robotec.ai)
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
  waitForAutowareStateToBeDriving([this]() { setAutowareEngage(true); });
#endif

#ifdef AUTOWARE_AUTO
  // TODO (Robotec.ai)
#endif
}
}  // namespace concealer
