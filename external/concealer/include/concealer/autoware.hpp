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

#ifndef CONCEALER__AUTOWARE_HPP_
#define CONCEALER__AUTOWARE_HPP_

// #define CONCEALER_ISOLATE_STANDARD_OUTPUT

#include <sys/wait.h>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_system_msgs/msg/emergency_state.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <chrono>
#include <concealer/continuous_transform_broadcaster.hpp>
#include <concealer/launch.hpp>
#include <concealer/task_queue.hpp>
#include <concealer/transition_assertion.hpp>
#include <concealer/utility/autoware_stream.hpp>
#include <concealer/utility/visibility.hpp>
#include <exception>
#include <future>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <limits>
#include <mutex>
#include <thread>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <utility>

namespace concealer
{
/* ---- NOTE -------------------------------------------------------------------
 *
 *  The magic class 'Autoware' is a class that makes it easy to work with
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
class Autoware : public rclcpp::Node, public ContinuousTransformBroadcaster<Autoware>
{
  friend class ContinuousTransformBroadcaster<Autoware>;

  std::atomic<bool> is_stop_requested = false;

  std::thread spinner;

  rclcpp::TimerBase::SharedPtr updater;

  std::atomic<bool> is_thrown = false;

  std::exception_ptr thrown;

  std::atomic<bool> is_autoware_exited = false;

  void checkAutowareProcess();

protected:
  const pid_t process_id = 0;

  int waitpid_options = 0;

  TaskQueue task_queue;

  bool initialize_was_called = false;

  geometry_msgs::msg::Accel current_acceleration;

  geometry_msgs::msg::Pose current_pose;

  geometry_msgs::msg::Twist current_twist;

  void stopRequest() noexcept { return is_stop_requested.store(true, std::memory_order_release); }

  bool isStopRequested() const noexcept
  {
    return is_stop_requested.load(std::memory_order_acquire);
  }

  virtual auto update() -> void = 0;

  // this method is purely virtual because different Autoware types are killed differently
  // currently, we are not sure why this is the case so detailed investigation is needed
  virtual void sendSIGINT() = 0;

  // method called in destructor of a derived class
  // because it is difficult to differentiate shutting down behavior in destructor of a base class
  void shutdownAutoware();

  void resetTimerCallback();

public:
  CONCEALER_PUBLIC explicit Autoware(pid_t pid = 0)
  : rclcpp::Node("concealer", "simulation", rclcpp::NodeOptions().use_global_arguments(false)),
    spinner([this]() {
      try {
        while (rclcpp::ok() and not isStopRequested()) {
          checkAutowareProcess();
          rclcpp::spin_some(get_node_base_interface());
          std::this_thread::yield();
        }
      } catch (...) {
        thrown = std::current_exception();
        is_thrown.store(true, std::memory_order_release);
      }
    }),
    process_id(pid)
  {
  }

  template <typename... Ts>
  CONCEALER_PUBLIC explicit Autoware(Ts &&... xs)
  : Autoware(ros2_launch(std::forward<decltype(xs)>(xs)...))
  {
  }

  ~Autoware() override = default;

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

  virtual auto getAcceleration() const -> double = 0;

  virtual auto getAutowareStateName() const -> std::string = 0;

  virtual auto getMinimumRiskManeuverBehaviorName() const -> std::string = 0;

  virtual auto getMinimumRiskManeuverStateName() const -> std::string = 0;

  virtual auto getEmergencyStateName() const -> std::string = 0;

  virtual auto getGearCommand() const -> autoware_auto_vehicle_msgs::msg::GearCommand;

  // returns -1.0 when gear is reverse and 1.0 otherwise
  virtual auto getGearSign() const -> double = 0;

  virtual auto getSteeringAngle() const -> double = 0;

  virtual auto getTurnIndicatorsCommand() const
    -> autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;

  virtual auto getVehicleCommand() const -> std::tuple<
    autoware_auto_control_msgs::msg::AckermannControlCommand,
    autoware_auto_vehicle_msgs::msg::GearCommand> = 0;

  virtual auto getVelocity() const -> double = 0;

  virtual auto getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray = 0;

  /*   */ auto initialized() const noexcept { return initialize_was_called; }

  // different autowares accept different initial target speed
  virtual auto restrictTargetSpeed(double) const -> double = 0;

  /*   */ auto rethrow() const noexcept(false) -> void;

  /*   */ auto set(const geometry_msgs::msg::Accel &) -> const geometry_msgs::msg::Accel &;

  /*   */ auto set(const geometry_msgs::msg::Pose &) -> const geometry_msgs::msg::Pose &;

  /*   */ auto set(const geometry_msgs::msg::Twist &) -> const geometry_msgs::msg::Twist &;

  virtual auto setCooperator(const std::string &) -> void = 0;

  virtual auto setVelocityLimit(double) -> void = 0;
};
}  // namespace concealer

#endif  // CONCEALER__AUTOWARE_HPP_
