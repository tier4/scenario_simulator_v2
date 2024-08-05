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

#ifndef CONCEALER__AUTOWARE_USER_HPP_
#define CONCEALER__AUTOWARE_USER_HPP_

// #define CONCEALER_ISOLATE_STANDARD_OUTPUT

#include <sys/wait.h>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <chrono>
#include <concealer/autoware_stream.hpp>
#include <concealer/launch.hpp>
#include <concealer/task_queue.hpp>
#include <concealer/transition_assertion.hpp>
#include <concealer/visibility.hpp>
#include <exception>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <utility>

namespace concealer
{
template <typename T>
class FieldOperatorApplicationFor;

/* ---- NOTE -------------------------------------------------------------------
 *
 *  The magic class 'FieldOperatorApplication' is a class that makes it easy to work with
 *  Autoware from C++. The main features of this class are as follows
 *
 *    (1) Launch Autoware in an independent process upon instantiation of the
 *        class.
 *    (2) Properly terminates the Autoware process started by the constructor
 *        upon destruction of the class.
 *    (3) Probably the simplest instructions to Autoware, consisting of
 *        initialize, plan, and engage.
 *
 * -------------------------------------------------------------------------- */
class FieldOperatorApplication : public rclcpp::Node
{
  std::atomic<bool> is_stop_requested = false;

  bool is_autoware_exited = false;

  auto checkAutowareProcess() -> void;

protected:
  const pid_t process_id = 0;

  TaskQueue task_queue;

  bool initialize_was_called = false;

  auto stopRequest() noexcept -> void;

  auto isStopRequested() const noexcept -> bool;

  // this method is purely virtual because different Autoware types are killed differently
  // currently, we are not sure why this is the case so detailed investigation is needed
  virtual auto sendSIGINT() -> void = 0;

  // method called in destructor of a derived class
  // because it is difficult to differentiate shutting down behavior in destructor of a base class
  auto shutdownAutoware() -> void;

public:
  CONCEALER_PUBLIC explicit FieldOperatorApplication(const pid_t = 0);

  template <typename... Ts>
  CONCEALER_PUBLIC explicit FieldOperatorApplication(Ts &&... xs)
  : FieldOperatorApplication(ros2_launch(std::forward<decltype(xs)>(xs)...))
  {
  }

  ~FieldOperatorApplication() override = default;

  auto spinSome() -> void;

  /* ---- NOTE -------------------------------------------------------------------
   *
   *  Send an engagement request to Autoware. If Autoware does not have an
   *  engagement equivalent, this operation can be nop (No operation).
   *
   * -------------------------------------------------------------------------- */
  virtual auto engage() -> void = 0;

  virtual auto engageable() const -> bool = 0;

  virtual auto engaged() const -> bool = 0;

  /* ---- NOTE -------------------------------------------------------------------
   *
   *  Send initial_pose to Autoware.
   *
   * -------------------------------------------------------------------------- */
  virtual auto initialize(const geometry_msgs::msg::Pose &) -> void = 0;

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
  virtual auto plan(const std::vector<geometry_msgs::msg::PoseStamped> &) -> void = 0;

  virtual auto clearRoute() -> void = 0;

  virtual auto getAutowareStateName() const -> std::string = 0;

  virtual auto getMinimumRiskManeuverBehaviorName() const -> std::string = 0;

  virtual auto getMinimumRiskManeuverStateName() const -> std::string = 0;

  virtual auto getEmergencyStateName() const -> std::string = 0;

  virtual auto getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray = 0;

  /*   */ auto initialized() const noexcept { return initialize_was_called; }

  virtual auto requestAutoModeForCooperation(const std::string &, bool) -> void = 0;

  // different autowares accept different initial target speed
  virtual auto restrictTargetSpeed(double) const -> double = 0;

  virtual auto getTurnIndicatorsCommand() const
    -> autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

  virtual auto rethrow() const noexcept(false) -> void;

  virtual auto sendCooperateCommand(const std::string &, const std::string &) -> void = 0;

  virtual auto setVelocityLimit(double) -> void = 0;

  virtual auto enableAutowareControl() -> void = 0;

  virtual auto disableAutowareControl() -> void = 0;
};
}  // namespace concealer

namespace autoware_vehicle_msgs::msg
{
auto operator<<(std::ostream &, const TurnIndicatorsCommand &) -> std::ostream &;

auto operator>>(std::istream &, TurnIndicatorsCommand &) -> std::istream &;
}  // namespace autoware_vehicle_msgs::msg

#endif  // CONCEALER__AUTOWARE_USER_HPP_
