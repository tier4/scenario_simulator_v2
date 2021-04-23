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

#include <awapi_accessor/autoware.hpp>

#define DEBUG_VALUE(...) \
  std::cout << "\x1b[32m" #__VA_ARGS__ " = " << (__VA_ARGS__) << "\x1b[0m" << std::endl

#define DEBUG_LINE() \
  std::cout << "\x1b[32m" << __FILE__ << ":" << __LINE__ << "\x1b[0m" << std::endl

namespace awapi
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
#if AUTOWARE_IV
  setCurrentControlMode();
  // setCurrentPose(current_pose);
  setCurrentShift(current_twist);
  setCurrentSteering(current_twist);
  setCurrentTurnSignal();
  setCurrentTwist(current_twist);
  setCurrentVelocity(current_twist);
  setLaneChangeApproval();
  setLocalizationTwist(current_twist);
  setTransform(current_pose);
  // setVehicleVelocity(parameters.performance.max_speed);
#elif AUTOWARE_AUTO
  // TODO (Robotec.ai)
#else
  static_assert(false, "");
#endif
}

void Autoware::initialize(const geometry_msgs::msg::Pose & initial_pose)
{
#if AUTOWARE_IV
  task_queue.delay([&]() {
    set(initial_pose);
    waitForAutowareStateToBeInitializingVehicle();
    waitForAutowareStateToBeWaitingForRoute([&]() { setInitialPose(initial_pose); });
  });
#elif AUTOWARE_AUTO
  task_queue.delay([&]() {
    // TODO (Robotec.ai)
  });
#else
  static_assert(false, "");
#endif
}

void Autoware::plan(
  const geometry_msgs::msg::PoseStamped & destination,
  const std::vector<geometry_msgs::msg::PoseStamped> & checkpoints)
{
  task_queue.delay([this, destination, checkpoints] {
    waitForAutowareStateToBeWaitingForRoute();  // NOTE: This is assertion.
    setGoalPose(destination);
    for (const auto & checkpoint : checkpoints) {
      setCheckpoint(checkpoint);
    }
    waitForAutowareStateToBePlanning();
    waitForAutowareStateToBeWaitingForEngage();  // NOTE: Autoware.IV 0.11.1 waits about 3 sec from the completion of Planning until the transition to WaitingForEngage.
  });
}

void Autoware::engage()
{
#if AUTOWARE_IV
  task_queue.delay([&] {  // TODO (yamacir-kit) REMOVE THIS DELAY!!!
    waitForAutowareStateToBeDriving([this]() { setAutowareEngage(true); });
  });
#elif AUTOWARE_AUTO
  // TODO (Robotec.ai)
#else
  static_assert(false, "");
#endif
}
}  // namespace awapi
