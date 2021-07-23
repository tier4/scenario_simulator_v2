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

#include <concealer/conversion.hpp>
#include <concealer/define_macro.hpp>

#include <concealer/continuous_transform_broadcaster.hpp>
#include <concealer/launch.hpp>
#include <concealer/task_queue.hpp>
#include <concealer/transition_assertion.hpp>
#include <concealer/utility/autoware_stream.hpp>
#include <concealer/utility/visibility.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <openscenario_msgs/msg/waypoints_array.hpp>

#include <chrono>
#include <exception>
#include <future>
#include <mutex>
#include <thread>
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
class Autoware : public rclcpp::Node,
                 public ContinuousTransformBroadcaster<Autoware>,
                 public TransitionAssertion<Autoware>
{
  friend class ContinuousTransformBroadcaster<Autoware>;
  friend class TransitionAssertion<Autoware>;

  mutable std::mutex mutex;

  std::promise<void> promise;

  std::thread spinner;

  const rclcpp::TimerBase::SharedPtr updater;

  std::exception_ptr thrown;

protected:
  const pid_t process_id;

  int waitpid_options = 0;

  TaskQueue task_queue;

  bool initialize_was_called = false;

  geometry_msgs::msg::Pose current_pose;

  geometry_msgs::msg::Twist current_twist;

  virtual void sendSIGINT() {
    std::cout << "Autoware::sendSIGINT" << std::endl;

    // TODO: not ok?
  };

  // method called in destructor of a derived class
  // because it is difficult to differentiate shutting down behavior in destructor of a base class
  void shutdownAutoware();

public:
  template <typename... Ts>
  CONCEALER_PUBLIC explicit Autoware(Ts &&... xs)
  : rclcpp::Node("concealer", "simulation", rclcpp::NodeOptions().use_global_arguments(false)),
    process_id(ros2_launch(std::forward<decltype(xs)>(xs)...)),
    spinner(
      [this](auto future) {
        while (rclcpp::ok() and
               future.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout) {
          try {
            rclcpp::spin_some(get_node_base_interface());
          } catch (...) {
            thrown = std::current_exception();
          }
        }
        RCLCPP_INFO_STREAM(
          get_logger(),
          "\x1b[32mShutting down Autoware: (1/3) Stoped publlishing/subscribing.\x1b[0m");
      },
      std::move(promise.get_future())),
    updater(create_wall_timer(std::chrono::milliseconds(5), [this]() { return update(); })) {}

  virtual ~Autoware() = default;

  /* ---- NOTE -------------------------------------------------------------------
   *
   *  Send an engagement request to Autoware. If Autoware does not have an
   *  engagement equivalent, this operation can be nop (No operation).
   *
   * -------------------------------------------------------------------------- */
  virtual void engage() = 0;

  /* ---- NOTE -------------------------------------------------------------------
   *
   *  Send initial_pose to Autoware.
   *
   * -------------------------------------------------------------------------- */
  virtual void initialize(const geometry_msgs::msg::Pose &) = 0;

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
  virtual void plan(const std::vector<geometry_msgs::msg::PoseStamped> &) = 0;

  virtual void update() = 0;

  virtual double getVelocity() const = 0;

  virtual double getSteeringAngle() const = 0;

  // returns -1.0 when gear is reverse and 1.0 otherwise
  virtual double getGearSign() const = 0;

  virtual openscenario_msgs::msg::WaypointsArray getWaypoints() const = 0;

  // different autowares accept different initial target speed
  virtual double restrictTargetSpeed(double value) const = 0;

  virtual std::string getAutowareStateMessage() const = 0;

  void rethrow() const noexcept(false);

  bool ready() const noexcept(false);

  auto initialized() const noexcept { return initialize_was_called; }

  auto lock() const { return std::unique_lock<std::mutex>(mutex); }

  auto set(const geometry_msgs::msg::Pose & pose) -> const auto & { return current_pose = pose; }

  auto set(const geometry_msgs::msg::Twist & twist) -> const auto & { return current_twist = twist; }
};
}  // namespace concealer

#include <concealer/undefine_macro.hpp>

#endif  // CONCEALER__AUTOWARE_HPP_
