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

#ifndef AWAPI_ACCESSOR__AUTOWARE_HPP_
#define AWAPI_ACCESSOR__AUTOWARE_HPP_

#define AUTOWARE_IV
// #define AUTOWARE_AUTO

#define AWAPI_CONCEALER_ISOLATE_STANDARD_OUTPUT false

#include <sys/wait.h>

#include <awapi_accessor/continuous_transform_broadcaster.hpp>
#include <awapi_accessor/fundamental_api.hpp>
#include <awapi_accessor/launch.hpp>
#include <awapi_accessor/miscellaneous_api.hpp>
#include <awapi_accessor/transition_assertion.hpp>
#include <awapi_accessor/utility/visibility.hpp>
#include <mutex>

namespace awapi
{
class Autoware : public rclcpp::Node,
                 public ContinuousTransformBroadcaster<Autoware>,
                 public FundamentalAPI<Autoware>,
                 public MiscellaneousAPI<Autoware>,
                 public TransitionAssertion<Autoware>
{
  friend class ContinuousTransformBroadcaster<Autoware>;
  friend class FundamentalAPI<Autoware>;
  friend class MiscellaneousAPI<Autoware>;
  friend class TransitionAssertion<Autoware>;

  std::mutex mutex;

  // TODO(yamacir-kit) MOVE INTO PRIVATE THIS!!!
  decltype(auto) lock() { return std::unique_lock<std::mutex>(mutex); }

  const pid_t process_id;

public:
  template <typename... Ts>
  AWAPI_ACCESSOR_PUBLIC explicit constexpr Autoware(Ts &&... xs)
  : rclcpp::Node(
      "autoware_concealer", "simulation", rclcpp::NodeOptions().use_global_arguments(false)),
    process_id(ros2_launch(std::forward<decltype(xs)>(xs)...))
  {
  }

  ~Autoware()
  {
    int status = 0;

    if (::kill(process_id, SIGINT) < 0 or ::waitpid(process_id, &status, WUNTRACED) < 0) {
      std::cout << std::system_error(errno, std::system_category()).what() << std::endl;
      std::exit(EXIT_FAILURE);
    }
  }

  /* ---- NOTE -----------------------------------------------------------------
   *
   *  Called for each execution frame of the simulator.
   *
   *  TODO(yamacir-kit) MOVE INTO class VehicleAPI.
   *
   * ------------------------------------------------------------------------ */
  void update(
    const geometry_msgs::msg::Pose & current_pose,
    const geometry_msgs::msg::Twist & current_twist = geometry_msgs::msg::Twist())
  {
    setCurrentControlMode();
    setCurrentPose(current_pose);
    setCurrentShift(current_twist);
    setCurrentSteering(current_twist);
    setCurrentTurnSignal();
    setCurrentTwist(current_twist);
    setCurrentVelocity(current_twist);
    setLaneChangeApproval();
    setLocalizationTwist(current_twist);
    setTransform(current_pose);
    // setVehicleVelocity(parameters.performance.max_speed);
  }

  void initialize(const geometry_msgs::msg::Pose & initial_pose)
  {
    waitForAutowareStateToBeInitializingVehicle([&]() { return update(initial_pose); });

    waitForAutowareStateToBeWaitingForRoute([&]() {
      setInitialPose(initial_pose);
      return update(initial_pose);
    });
  }
};
}  // namespace awapi

#include <awapi_accessor/undefine_macro.hpp>

#endif  // AWAPI_ACCESSOR__AUTOWARE_HPP_
