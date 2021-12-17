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

#ifndef CONCEALER__AUTOWARE_HPP_
#define CONCEALER__AUTOWARE_HPP_

// #define CONCEALER_ISOLATE_STANDARD_OUTPUT

#include <sys/wait.h>

#include <autoware_vehicle_msgs/msg/vehicle_command.hpp>
#include <chrono>
#include <concealer/continuous_transform_broadcaster.hpp>
#include <concealer/conversion.hpp>
#include <concealer/define_macro.hpp>
#include <concealer/launch.hpp>
#include <concealer/task_queue.hpp>
#include <concealer/transition_assertion.hpp>
#include <concealer/utility/autoware_stream.hpp>
#include <concealer/utility/visibility.hpp>
#include <exception>
#include <future>
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

  mutable std::mutex mutex;

  std::promise<void> promise;

  std::future<void> future;

  std::thread spinner;

  rclcpp::TimerBase::SharedPtr updater;

  std::exception_ptr thrown;

protected:
  const pid_t process_id = 0;

  int waitpid_options = 0;

  TaskQueue task_queue;

  bool initialize_was_called = false;

  geometry_msgs::msg::Pose current_pose;

  geometry_msgs::msg::Twist current_twist;

  double current_upper_bound_speed = std::numeric_limits<double>::max();

  auto currentFuture() -> auto & { return future; }

  // this method is purely virtual because different Autoware types are killed differently
  // currently, we are not sure why this is the case so detailed investigation is needed
  virtual void sendSIGINT() = 0;

  // method called in destructor of a derived class
  // because it is difficult to differentiate shutting down behavior in destructor of a base class
  void shutdownAutoware();

  void resetTimerCallback();

public:
  CONCEALER_PUBLIC explicit Autoware()
  : rclcpp::Node("concealer", "simulation", rclcpp::NodeOptions().use_global_arguments(false)),
    future(std::move(promise.get_future())),
    spinner([this]() {
      while (rclcpp::ok() and currentFuture().wait_for(std::chrono::milliseconds(1)) ==
                                std::future_status::timeout) {
        try {
          rclcpp::spin_some(get_node_base_interface());
        } catch (...) {
          thrown = std::current_exception();
        }
      }
    })
  {
  }

  template <typename... Ts>
  CONCEALER_PUBLIC explicit Autoware(Ts &&... xs)
  : rclcpp::Node("concealer", "simulation", rclcpp::NodeOptions().use_global_arguments(false)),
    future(std::move(promise.get_future())),
    spinner([this]() {
      while (rclcpp::ok() and currentFuture().wait_for(std::chrono::milliseconds(1)) ==
                                std::future_status::timeout) {
        try {
          rclcpp::spin_some(get_node_base_interface());
        } catch (...) {
          thrown = std::current_exception();
        }
      }
      RCLCPP_INFO_STREAM(
        get_logger(),
        "\x1b[32mShutting down Autoware: (1/3) Stopped publishing/subscribing.\x1b[0m");
    }),
    process_id(ros2_launch(std::forward<decltype(xs)>(xs)...))
  {
  }

  virtual ~Autoware() = default;

  /* ---- NOTE -------------------------------------------------------------------
   *
   *  Send an engagement request to Autoware. If Autoware does not have an
   *  engagement equivalent, this operation can be nop (No operation).
   *
   * -------------------------------------------------------------------------- */
  virtual auto engage() -> void = 0;

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

  virtual auto update() -> void = 0;

  virtual auto getAcceleration() const -> double = 0;

  virtual auto getAutowareStateMessage() const -> std::string = 0;

  // returns -1.0 when gear is reverse and 1.0 otherwise
  virtual auto getGearSign() const -> double = 0;

  virtual auto getSteeringAngle() const -> double = 0;

  virtual auto getVehicleCommand() const -> autoware_vehicle_msgs::msg::VehicleCommand = 0;

  virtual auto getVelocity() const -> double = 0;

  virtual auto getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray = 0;

  /*   */ auto initialized() const noexcept { return initialize_was_called; }

  /*   */ auto lock() const { return std::unique_lock<std::mutex>(mutex); }

  /*   */ auto ready() const noexcept(false) -> bool;

  // different autowares accept different initial target speed
  virtual auto restrictTargetSpeed(double) const -> double = 0;

  /*   */ auto rethrow() const noexcept(false) -> void;

  /*   */ auto set(const geometry_msgs::msg::Pose &) -> const geometry_msgs::msg::Pose &;

  /*   */ auto set(const geometry_msgs::msg::Twist &) -> const geometry_msgs::msg::Twist &;

  /*   */ auto setUpperBoundSpeed(double) -> double;
};
}  // namespace concealer

#include <concealer/undefine_macro.hpp>

#endif  // CONCEALER__AUTOWARE_HPP_
